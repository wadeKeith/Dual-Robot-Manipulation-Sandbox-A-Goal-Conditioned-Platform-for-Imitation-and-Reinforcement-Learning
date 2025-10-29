import numpy as np
import cv2


out = cv2.VideoWriter('outpy.mp4v', cv2.VideoWriter_fourcc(*'mp4v'), 10, (1280,720))

obs = np.load('obs.npy', allow_pickle=True)

for i in range(len(obs)):
    out.write(obs[i]['img'])

out.release()

print('a')








