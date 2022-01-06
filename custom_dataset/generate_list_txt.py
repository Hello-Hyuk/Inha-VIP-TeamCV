import os

# dir_path = "data/train"
# dir_path = "data/test"
dir_path = "9"
data = []
for (root, directories, files) in os.walk(dir_path):
    for file in files:
        if '.jpg' in file:
            file_path = os.path.join(root, file)
            #print(file_path)
            data.append(file_path)

# train.txt / test.txt
data_list = open(r'test.txt','w')

for i in data:
    data_list.write(i + '\n')

data_list.close()