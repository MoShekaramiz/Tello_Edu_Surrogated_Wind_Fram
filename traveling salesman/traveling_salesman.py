import math
import numpy as np
import matplotlib.pyplot as plt
from random import randint, random
import time
import threading


start = time.time()
coordinates = [(57, 80), (16, 42), (14, 72), (47, 49),
               (90, 80), (55, 35), (3, 7), (5, 59), (80, 91),
               (45, 19), (38, 43), (78, 74), (36, 3), (53, 94),
               (71, 76), (87, 55), (32, 18), (65, 49), (97, 51),
               (7, 99)]

xpos = np.array([57, 16, 14, 47, 90, 55, 3, 5, 80, 45, 38, 78, 36, 53, 71, 87, 32, 65, 97, 7])
xpos = np.append(xpos, xpos[0])
ypos = np.array([80, 42, 72, 49, 80, 35, 7, 59, 91, 19, 43, 74, 3, 94, 76, 55, 18, 49, 51, 99])
ypos = np.append(ypos, ypos[0])
data = np.array([xpos, ypos], np.int32)

class TravelingSalesman():
    def __init__(self):
        self.path1 = np.copy(data)
        self.e1 = energy_calc(data)
        self.e_min = self.e1
        self.path_min = data
        print(data)
        print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>", self.e1)

        T = 2000

        while T > 1:
            thread1 = EnergyThreading(np.copy(self.path1), self.e1, T, np.copy(self.path_min))
            thread2 = EnergyThreading(np.copy(self.path1), self.e1, T, np.copy(self.path_min))
            thread3 = EnergyThreading(np.copy(self.path1), self.e1, T, np.copy(self.path_min))
            thread4 = EnergyThreading(np.copy(self.path1), self.e1, T, np.copy(self.path_min))
            threads = [thread1, thread2, thread3, thread4]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
            for thread in threads:
                if thread.get_energy() < self.e_min:
                    print(round(self.e_min), round(thread.get_energy()))
                    self.e_min = thread.get_energy()
                    self.path1 = thread.get_path1()
                    self.path_min = thread.get_path_min()
                    self.e1 = thread.get_e1()
            T = 0.99 * T
            # for i in range(400):
            #     if e1 < e_min:
            #         e_min = e1
            #     if e_iter == []:
            #             e_iter = e1
            #     else:
            #         e_iter = np.append(e_iter, e1) # line 32
            #     rand_index = randint(1, xpos.size-3)
            #     path2 = np.copy(path1)# line 34
            #     path2[0, rand_index] = path1[0, rand_index+1]
            #     path2[1, rand_index] = path1[1, rand_index+1]
            #     path2[0, rand_index+1] = path1[0, rand_index]
            #     path2[1, rand_index+1] = path1[1, rand_index]
            #     e2 = energy_calc(path2)

            #     if e2 < e1:
            #         e1 = e2
            #         path1 = np.copy(path2)
            #     else:
            #         c1 = math.exp(-(e2 - e1)/T)
            #         c2 = random()
            #         if temp2 == []:
            #             temp2 = c1
            #         else:
            #             temp2 = np.append(temp2, c1)

            #         if c1 > c2:
            #             path1 = np.copy(path2)
            #             e1 = e2
                    
            # T = 0.99 * T
            #print(T)
        print(energy_calc(self.path_min))
        print("Run time: ", time.time() - start)
        plt.plot(self.path_min[0], self.path_min[1])
        plt.show()

class EnergyThreading(threading.Thread):  
    def __init__(self, path1, e1, T, path_min):
        # self.energy and self.path1 will have the final energy and dataset needed
        threading.Thread.__init__(self)
        self.path1 = np.copy(path1)
        self.e1 = e1 #energy_calc(self.path1)
        self.energy = self.e1
        self.path_min = path_min
        for i in range(400):
                if self.e1 < self.energy:
                    self.energy = self.e1
                    self.path_min = np.copy(self.path1)
                rand_index = randint(1, xpos.size-3)
                self.path2 = np.copy(self.path1)# line 34
                self.path2[0, rand_index] = self.path1[0, rand_index+1]
                self.path2[1, rand_index] = self.path1[1, rand_index+1]
                self.path2[0, rand_index+1] = self.path1[0, rand_index]
                self.path2[1, rand_index+1] = self.path1[1, rand_index]
                e2 = energy_calc(self.path2)

                if e2 < self.e1:
                    self.e1 = e2
                    self.path1 = np.copy(self.path2)
                else:
                    c1 = math.exp(-(e2 - self.e1)/T)
                    c2 = random()

                    if c1 > c2:
                        self.path1 = np.copy(self.path2)
                        self.e1 = e2
        
    def get_energy(self):
        return self.energy
    
    def get_path1(self):
        return np.copy(self.path1)

    def get_path_min(self):
        return np.copy(self.path_min)

    def get_e1(self):
        return self.e1

def energy_calc(data):
    energy = 0
    for i in range(int((data.size)/2)-1):
        energy += math.sqrt((data[0][i] - data[0][i+1])**2 + (data[1][i] - data[1][i+1])**2)
    return energy

if __name__ == "__main__":
    TravelingSalesman()