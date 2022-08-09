
distance = 100
with open('CARspeed.txt') as f:
    Cspeeds = f.readlines()
Ctimes = []
for i in range(len(Cspeeds)) :
    Cspeed_ = Cspeeds[i].split("\t")
    j = 0
    Ctime = []
    while Cspeed_[j] != "\n" :
        time = distance/float(Cspeed_[j])
        Ctime.append(time)
    Ctimes.append(Ctime)
print(Ctimes)