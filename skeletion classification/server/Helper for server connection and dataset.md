To connect to the server 3090

Add following lines to config.json in vac remote

```terminal
Host imaging_lab
    HostName 166.111.154.17
    User lihaowei
    Port 58022
    IdentityFile /Users/lihaowei/semesters/else/id_rsa 
```

"/Users/lihaowei/semesters/else/id_rsa " should be changed to the file path of the id_rsa in your pc. File id_rsa and id_rsa.pub should be put in one folder.

To train model, you should use environment lab, activate by

``` terminal
conda activate lab
```

Notice that when using Jupyter notebook, restart the notebook before close it to avoid dead programme. When training, run in terminal.

Dataset is stored in:

``` terminal
/mnt/disk/lihaowei/alphapose+GCN/st_gcn/code/shift_gcn/Shift-GCN-master/Shift-GCN-master/data/nturgbd_raw/nturgb+d_skeletons
```

Processed data is stored in

``` terminal
/mnt/disk/lihaowei/alphapose+GCN/st_gcn/code/shift_gcn/Shift-GCN-master/Shift-GCN-master/data/ntu
```

Detailed data description can be found in https://blog.csdn.net/Hren0412/article/details/89495678

Other datasets can be found in website: http://rose1.ntu.edu.sg/Datasets/actionRecognition.asp

Drag directly to the bottom for download. id and password are:

``` 
Login Id: R6072
Password: 1B63E403
```



