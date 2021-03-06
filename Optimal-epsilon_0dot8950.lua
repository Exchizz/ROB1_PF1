wc = rws.getRobWorkStudio():getWorkCell()
state = wc:getDefaultState()
device = wc:findDevice("KukaKr16")
gripper = wc:findFrame("Tool")
bottle = wc:findFrame("Bottle")
table = wc:findFrame("Table")
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
setQ({-3.142, -0.827, -3.002, -3.143, 0.099, -1.573})
attach(bottle,gripper)
setQ({-2.83058, -0.654302, -2.6491, -3.82141, 0.00899654, -1.8582})
setQ({-2.12217, -0.711182, -2.26544, -4.09368, -0.24186, -1.96637})
setQ({-1.73655, -0.39749, -2.33564, -3.97644, -0.0619279, -1.25724})
setQ({-1.38122, -0.353757, -2.06821, -3.54933, 0.0109747, -0.614121})
setQ({-1.0259, -0.310024, -1.80079, -3.12221, 0.0838773, 0.0289946})
setQ({-0.670573, -0.266291, -1.53336, -2.6951, 0.15678, 0.672111})
setQ({-0.264711, -0.0197738, -1.9638, -3.11813, 0.614257, 0.716959})
setQ({0.358856, 0.271508, -1.91729, -2.86162, 1.0223, 0.412179})
setQ({0.769746, 0.212615, -1.81835, -2.89071, 0.388731, -0.0533178})
setQ({1.18064, 0.153722, -1.71941, -2.9198, -0.244833, -0.518815})
setQ({1.30615, 0.135731, -1.68919, -2.92869, -0.438374, -0.661015})
setQ({0.87953, 0.234582, -1.13933, -2.81488, -0.0764407, -0.257349})
setQ({0.812936, 0.0488688, -1.06123, -2.1058, -0.010086, 0.241452})
setQ({0.964823, -0.0405768, -0.727642, -2.30159, -0.0634158, 1.02726})
setQ({0.883825, -0.19858, -0.482285, -1.4732, -0.141289, 1.15772})
setQ({0.821662, -0.277471, -0.210399, -1.2429, 0.223727, 1.88624})
setQ({0.933904, -0.185155, -0.166274, -1.39825, 0.274016, 2.75301})
setQ({0.838493, -0.080556, -0.397601, -0.881134, 0.466398, 3.40343})
setQ({1.14028, -0.0586149, -0.232168, -0.235922, 0.688314, 3.86879})
setQ({1.571, 0.006, 0.03, 0.153, 0.762, 4.49})
attach(bottle,table)
