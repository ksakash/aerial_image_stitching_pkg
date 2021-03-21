#!/usr/bin/env python3

import pyclipper

subj = (
        ((0,0), (10,0), (10,10), (0,10)),
)
clip = ((5,5), (15,5), (15,15), (5,15))

pc = pyclipper.Pyclipper ()
pc.AddPath (clip, pyclipper.PT_CLIP, True)
pc.AddPaths (subj, pyclipper.PT_SUBJECT, True)

sol = pc.Execute (pyclipper.CT_INTERSECTION, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)

print (sol)

