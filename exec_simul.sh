#/bin/sh
LOADER="/home/ezpeleta/aplicaciones/renew2.6/loader.jar"
MODEL="multi_wf_7.sns"
SYSTEM_NET="execute_experiment"

java -Dde.renew.remote.enable=true -jar ${LOADER} startsimulation ${MODEL} ${SYSTEM_NET}