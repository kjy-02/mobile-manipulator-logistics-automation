#!/usr/bin/env python3
import copy
import pathlib
import sys
import unittest
import yaml
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'scripts'))
from validate_integration_config import validate

class ConfigValidatorTest(unittest.TestCase):
    def setUp(self):
        root=pathlib.Path(__file__).resolve().parents[1]
        self.dwa=yaml.safe_load((root/'config/synthetic_dwa.yaml').read_text())
        self.adapter=yaml.safe_load((root/'config/cloud_adapter_reference.yaml').read_text())
    def test_valid(self):
        self.assertEqual(validate(self.dwa,self.adapter),[])
    def test_division_and_sample_crashes_rejected(self):
        for key,value in (('HZ',0),('PREDICT_TIME',float('nan')),('SIM_PERIOD',0),
                          ('VELOCITY_SAMPLES',1),('YAWRATE_SAMPLES',1),('SIM_TIME_SAMPLES',0)):
            data=copy.deepcopy(self.dwa); data[key]=value
            self.assertTrue(validate(data,self.adapter),(key,value))
    def test_scan_float32_boundaries(self):
        for value in (0,1e-50,1e50,.2,float('nan')):
            adapter=copy.deepcopy(self.adapter); adapter['scan_angle_increment']=value
            self.assertTrue(validate(self.dwa,adapter),value)
    def test_bounds_and_frames(self):
        dwa=copy.deepcopy(self.dwa); dwa['TARGET_VELOCITY']=1
        self.assertTrue(validate(dwa,self.adapter))
        adapter=copy.deepcopy(self.adapter); adapter['target_frame']='wrong'
        self.assertTrue(validate(self.dwa,adapter))
    def test_resource_limits(self):
        dwa=copy.deepcopy(self.dwa); dwa['YAWRATE_SAMPLES']=100000
        self.assertTrue(validate(dwa,self.adapter))
        adapter=copy.deepcopy(self.adapter); adapter['map_resolution']=1e-5
        self.assertTrue(validate(self.dwa,adapter))

if __name__=='__main__':
    unittest.main()
