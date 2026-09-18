"""Timeout recovery must preserve real parameter values and fail closed."""
import sys
from pathlib import Path
from unittest.mock import Mock,patch
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from trajectory_evaluation import read_pixel_offset

if __name__=='__main__':
    def reply(kind=3,value=.5):
        future=Mock();future.done.return_value=True
        future.result.return_value=SimpleNamespace(values=[SimpleNamespace(type=kind,double_value=value)])
        return future
    with patch('rclpy.spin_until_future_complete'):
        client=Mock();client.wait_for_services.return_value=True
        late=Mock();late.done.return_value=False
        client.get_parameters.side_effect=[late,reply()];events=[]
        assert read_pixel_offset(None,client,events)==.5 and late.cancel.called
        assert [e['status'] for e in events]==['response_timeout','success']
        client.get_parameters.side_effect=[late,late]
        try:read_pixel_offset(None,client,[])
        except TimeoutError:pass
        else:raise AssertionError('Unbounded or silent timeout fallback')
        for kind,value in [(2,.5),(3,float('nan')),(3,2.)]:
            client.get_parameters.side_effect=[reply(kind,value)];events=[]
            try:read_pixel_offset(None,client,events)
            except ValueError:pass
            else:raise AssertionError('Invalid calibration accepted')
            assert len(events)==1 and events[0]['status']=='invalid_value'
    print('PASS bounded timeout retry, cancellation, exact calibration and no bad-value fallback')
