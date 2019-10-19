#!/bin/bash
echo Building...
/usr/bin/env python3 $(which scons) build profile=debug
echo Done!