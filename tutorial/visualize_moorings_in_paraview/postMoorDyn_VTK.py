
```
    Chen, H., & Hall, M. (2022). CFD simulation of floating body motion with mooring dynamics: 
        Coupling MoorDyn with OpenFOAM. Applied Ocean Research, 124, 103210.
        https://doi.org/10.1016/j.apor.2022.103210
```

import numpy as np
import json 
import os

# post-process MoorDyn output: generate mooring.vtk.series

## ------------------ Change input here ---------------------
tstart = 0
tstop = 16.1
tinterval = 0.05

nlines = 4

mdversion = 1

prefix = '../deformMesh/moorDynR/Mooring/'


## ------------------ End input -----------------------------

if mdversion == 1:
    # MoorDyn v1
    filenames = ['Line{}.out'.format(i) for i in range(1,nlines+1)]
else:
    # MoorDyn v2
    filenames = ['linesWu2_Line{}.out'.format(i) for i in range(1,nlines+1)]

print(filenames)

tselect = np.arange(tstart, tstop, tinterval)

# load the first output file
data = np.loadtxt(prefix+filenames[0], skiprows=2)

time = data[:,0]
# Check if each time instant has one or more entries
# Use the second time instant, since the first one is time 0
nEntry = 1
for i in range(2, 10):
  if time[i] == time[1]:
    nEntry = nEntry +1
  else:
    break

print('Each time instant has {} entry (entries).'.format(nEntry))
if nEntry > 1:
  reduceRow = [0, *range(nEntry, len(data[:,0]), nEntry)]
  #print(reduceRow[:5])
  
  data = data[reduceRow,:]

  print('Data array rows reduced.')

  time = data[:,0]

print("Time range in output file: ", time[0], " - ", time[-1])

npoints = int(len(data[0,1:])/3)
print("Number of lines, number of points per line: ", nlines, npoints)

#!mkdir mooringVTK

os.mkdir(prefix+"mooringVTK/")


if nlines>1:
  # load other files
  if nEntry > 1:
    otherData = [np.loadtxt(prefix+f1, skiprows=2)[reduceRow,:] for f1 in filenames[1:]]
  else:
    otherData = [np.loadtxt(prefix+f1, skiprows=2) for f1 in filenames[1:]]


vtkDict = []

print("Time marching ...")

for i, ti in enumerate(time):
  if abs(tselect-ti).min() < 1e-5:
  
  #if ti in tselect:
    #ind = np.where(tselect == ti)[0]
    ind = np.where(abs(tselect-ti)==abs(tselect-ti).min())[0]
    
    vtkName = "mooring{}.vtk".format(ind[0])
    #print(ti, ' ', end='')
    #print(ti, vtkName)
    
    file1 = open(prefix+"mooringVTK/" + vtkName, "w")

    dict1 = { "name" : vtkName, "time" : ti}
    vtkDict.append(dict1)

    #https://www.visitusers.org/index.php?title=ASCII_VTK_Files
    # Writing header
    header = [
              "# vtk DataFile Version 3.0\n", 
              "vtk output\n",
              "ASCII\n",
              "DATASET POLYDATA\n"
    ]
    file1.writelines(header)
    
    #POINTS 12 float
    file1.write("\nPOINTS {} float\n".format(nlines*npoints))

    points = np.reshape(data[i,1:], (-1,3))
    #print(points)

    for p in points:
      file1.write(' '.join('{:.3f}'.format(px) for px in p))
      file1.write('\n')

    if nlines>1:
      #write points for other mooring lines
      for data1 in otherData:
        points1 = np.reshape(data1[i,1:], (-1,3))
        
        for p in points1:
          file1.write(' '.join('{:.3f}'.format(px) for px in p))
          file1.write('\n')

    file1.write("\nLINES {} {}\n".format(nlines, nlines*(npoints+1)))

    for li in range(nlines):
      pindex = np.arange(li*npoints, (li+1)*npoints)
      file1.write(str(npoints)+' ')
      file1.write(" ".join(str(p) for p in pindex))
      file1.write('\n')

    file1.close()

vtkSeries = {
  "file-series-version" : "1.0",
  "files" : vtkDict
}

with open(prefix+"mooringVTK/mooring.vtk.series", "w") as outfile: 
    json.dump(vtkSeries, outfile,  indent = 4)

print('\nGenerated mooring#.vtk and mooring.vtk.series in folder: {}mooringVTK'.format(prefix))

# Zip all the output files
#!zip -r mooringVTk.zip mooringVTK/mooring*.vtk*    
