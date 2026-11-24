#include "FeedbackLease.hh"
#include <cassert>
int main() {
  mooncar::FeedbackLease lease;
  assert(!lease.Fresh(0));
  assert(lease.Receive("1 1 1 0 0.96 1"));
  assert(lease.Fresh(1000000000));assert(lease.Fresh(1250000000));
  assert(!lease.Fresh(1250000001));assert(!lease.Fresh(999999999));
  assert(!lease.Receive("1 1 1 0 0.96 1"));assert(!lease.Fresh(1000000000));
  assert(lease.Receive("1 2 1 100000000 1.01 0"));assert(!lease.Fresh(1100000000));
  assert(!lease.Receive("1 3 1 50000000 1.01 1"));
  for (const auto *text : {"1 -1 1 0 0.96 1","1 3 1 200000000 nan 1", "1 3 1 200000000 1.1 1",
      "2 3 1 200000000 0.96 1","1 3 1 200000000 0.96 1 extra","1 3 1 1000000000 0.96 1"})
    assert(!lease.Receive(text));
  assert(lease.Receive("1 3 1 200000000 0.99 1"));assert(lease.Fresh(1200000000));
}
