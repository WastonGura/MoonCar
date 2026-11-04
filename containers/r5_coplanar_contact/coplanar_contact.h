// Test-only ODE 0.16.2 cylinder/trimesh repair. Included inside the OPCODE
// collider translation unit, after the broadphase. No state or force changes.
#include <algorithm>
#include <vector>

namespace mooncar_coplanar {
struct Interval { dReal lo, hi; int triangle; };

// Handle only an entirely coplanar broadphase patch that covers the whole
// cylinder support generator. Boundary, hole, crease, cap and one-end contact
// cases deliberately retain the original ODE collider.
static bool collide(dxGeom *cylinder, dxTriMesh *mesh, const int *triangles,
                    int count, int flags, dContactGeom *contacts, int skip,
                    int &written) {
    if (count < 2 || count > 256 || mesh->m_Callback != NULL) return false;
    const dReal tolerance = REAL(1e-12); // floating-point geometry check, metres
    const dReal *center = dGeomGetPosition(cylinder);
    const dReal *rotation = dGeomGetRotation(cylinder);
    dVector3 axis = {rotation[2], rotation[6], rotation[10], 0};
    dReal radius, length;
    dGeomCylinderGetParams(cylinder, &radius, &length);
    dVector3 triangle[3], normal, e0, e1;
    mesh->fetchMeshTriangle(triangle, triangles[0], *reinterpret_cast<const dVector3 *>(dGeomGetPosition(mesh)), dGeomGetRotation(mesh));
    dSubtractVectors3(e0, triangle[1], triangle[0]);
    dSubtractVectors3(e1, triangle[2], triangle[0]);
    dCalcVectorCross3(normal, e0, e1);
    if (!dSafeNormalize3(normal)) return false;
    const dReal plane = dCalcVectorDot3(normal, triangle[0]);
    if (dCalcVectorDot3(normal, center) - plane <= 0) return false;
    const dReal axial = dCalcVectorDot3(normal, axis);
    if (dFabs(axial) >= REAL(0.9)) return false; // preserve cap collider
    dVector3 radial, points[2];
    for (int k=0; k<3; ++k) radial[k] = normal[k] - axis[k]*axial;
    if (!dSafeNormalize3(radial)) return false;
    dReal depths[2];
    for (int p=0; p<2; ++p) {
        for (int k=0; k<3; ++k)
            points[p][k] = center[k] - radius*radial[k] + (p ? 1 : -1)*length*REAL(0.5)*axis[k];
        depths[p] = plane - dCalcVectorDot3(normal, points[p]);
        // Do not clamp separated endpoints into contact.
        if (depths[p] < 0) return false;
    }
    std::vector<Interval> coverage;
    coverage.reserve(count);
    const dVector3 &position = *reinterpret_cast<const dVector3 *>(dGeomGetPosition(mesh));
    const dReal *meshRotation = dGeomGetRotation(mesh);
    for (int i=0; i<count; ++i) {
        mesh->fetchMeshTriangle(triangle, triangles[i], position, meshRotation);
        dSubtractVectors3(e0, triangle[1], triangle[0]);
        dSubtractVectors3(e1, triangle[2], triangle[0]);
        dVector3 faceNormal;
        dCalcVectorCross3(faceNormal, e0, e1);
        if (!dSafeNormalize3(faceNormal)) return false;
        for (int k=0; k<3; ++k)
            if (dFabs(faceNormal[k]-normal[k]) > tolerance) return false;
        for (int v=0; v<3; ++v)
            if (dFabs(dCalcVectorDot3(normal, triangle[v])-plane) > tolerance) return false;
        dReal lo=0, hi=1;
        for (int edge=0; edge<3 && lo<=hi; ++edge) {
            dVector3 direction, inward, delta;
            dSubtractVectors3(direction, triangle[(edge+1)%3], triangle[edge]);
            dCalcVectorCross3(inward, normal, direction);
            if (!dSafeNormalize3(inward)) return false;
            dSubtractVectors3(delta, points[0], triangle[edge]);
            const dReal a=dCalcVectorDot3(inward, delta);
            dSubtractVectors3(delta, points[1], points[0]);
            const dReal b=dCalcVectorDot3(inward, delta);
            if (dFabs(b) <= tolerance) {
                if (a < -tolerance) { lo=1; hi=0; }
            } else if (b>0) lo=std::max(lo, -a/b);
            else hi=std::min(hi, -a/b);
        }
        if (lo<=hi) coverage.push_back({lo, hi, triangles[i]});
    }
    std::sort(coverage.begin(), coverage.end(), [](const Interval &a, const Interval &b) { return a.lo<b.lo; });
    dReal end=0;
    for (const auto &interval : coverage) {
        if (interval.lo > end+tolerance) return false; // true uncovered gap
        end=std::max(end, interval.hi);
    }
    if (end < 1-tolerance) return false;
    written=0;
    // Both points are actual cylinder-surface points, not plane projections.
    const int first=depths[0]>=depths[1] ? 0 : 1;
    for (int j=0; j<2 && written<(flags & NUMC_MASK); ++j) {
        const int p=(flags & NUMC_MASK)==1 ? first : j;
        dContactGeom *contact=SAFECONTACT(flags, contacts, written, skip);
        for (int k=0; k<3; ++k) { contact->pos[k]=points[p][k]; contact->normal[k]=normal[k]; }
        contact->depth=depths[p]; contact->g1=cylinder; contact->g2=mesh;
        contact->side1=-1; contact->side2=-1;
        for (const auto &interval : coverage)
            if (interval.lo<=p+tolerance && interval.hi>=p-tolerance) { contact->side2=interval.triangle; break; }
        if (contact->side2<0) return false;
        ++written;
    }
    return true;
}
} // namespace mooncar_coplanar
