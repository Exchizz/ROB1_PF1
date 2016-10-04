#!/bin/bash


cat << EOF
wc = rws.getRobWorkStudio():getWorkCell()
state = wc:getDefaultState()
device = wc:findDevice("KukaKr16")
gripper = wc:findFrame("Tool");
bottle = wc:findFrame("Bottle");
table = wc:findFrame("Table");

function setQ(q)
qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])
device:setQ(qq,state)
rws.getRobWorkStudio():setState(state)
rw.sleep(0.1)
end

function attach(obj, tool)
rw.gripFrame(obj, tool, state)
rws.getRobWorkStudio():setState(state)
rw.sleep(0.1)
end

EOF


echo $var
OUTPUT=$(./PF1  2>&1 | sed 's/Q\[6]/setQ(/' | sed 's/}/})/' | grep 'setQ' | grep -v 'Planning')


echo "$OUTPUT" | head -1
echo "attach(bottle,gripper)"
echo "$OUTPUT" | tail -n+2
echo "attach(bottle,table)"
