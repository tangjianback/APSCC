# HPS simulation
1 cd simulation
2 chmod +x waf
3 ./waf configure --build-profile=optimized
4 python run.py --cc timely --bw 100 --topo topology
