#!/bin/bash

export XPCF_MODULE_ROOT=~/.remaken/packages/linux-gcc
echo "XPCF_MODULE_ROOT=$XPCF_MODULE_ROOT"

ld_library_path="./"
for modulePath in $(grep -o "\$XPCF_MODULE_ROOT.*lib" $1_conf.xml)
do
   modulePath=${modulePath/"\$XPCF_MODULE_ROOT"/${XPCF_MODULE_ROOT}}
   if ! [[ $ld_library_path =~ "$modulePath/x86_64/shared/release" ]]
   then
      ld_library_path=$ld_library_path:$modulePath/x86_64/shared/release
   fi 
done

echo "LD_LIBRARY_PATH=$ld_library_path $1"
LD_LIBRARY_PATH=$ld_library_path $1
