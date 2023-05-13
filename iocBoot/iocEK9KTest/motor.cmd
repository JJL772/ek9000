#!/usr/bin/env bash

caput IOC:m1.SSET 1
#caput IOC:m1.SET 1
caput IOC:m1.DVAL 10
caput IOC:m1.SPMG 3

# vim: syn=bash
