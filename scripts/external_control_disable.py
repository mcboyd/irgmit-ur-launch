#!/usr/bin/env python
import os
os.system("echo y | plink root@172.16.0.10 -pw easybot \"{ echo \"stop\"; echo \"quit\"; } | nc 127.0.0.1 29999\"")
