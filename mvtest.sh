#!/bin/bash
bin/client set 10 20 0 8 0 0 1000
echo "prior:"
bin/client get 10
bin/client step 10
