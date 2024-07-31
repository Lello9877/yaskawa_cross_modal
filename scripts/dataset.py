# import rospy
# import open3d as o3d
# import numpy as np
# import torch
# from torch.utils.data import Dataset, DataLoader
# import torch.nn as nn
# import torch.optim as optim
# import time
# import os

# class PointCloudDataset(Dataset):
#     def __init__(self, file_paths, labels, num_points):
#         self.file_paths = file_paths
#         self.labels = labels
#         self.num_points = num_points

#     def __len__(self):
#         return len(self.file_paths)

#     def __getitem__(self, idx):
#         point_cloud = self.load_point_cloud(self.file_paths[idx])
#         label = self.labels[idx]
#         point_cloud = self.random_sample(point_cloud, self.num_points)
#         return torch.tensor(point_cloud, dtype=torch.float32).permute(1, 0), label

#     @staticmethod
#     def load_point_cloud(file_path):
#         pcd = o3d.io.read_point_cloud(file_path)
#         return np.asarray(pcd.points)
    
#     def random_sample(self, point_cloud, num_points):
#         if point_cloud.shape[0] >= num_points:
#             indices = np.random.choice(point_cloud.shape[0], num_points, replace=False)
#         else:
#             indices = np.random.choice(point_cloud.shape[0], num_points, replace=True)
#         return point_cloud[indices]
    
# class PointNet(nn.Module):
#     def __init__(self, num_classes=2):
#         super(PointNet, self).__init__()
#         self.conv1 = nn.Conv1d(3, 64, 1)
#         self.conv2 = nn.Conv1d(64, 128, 1)
#         self.conv3 = nn.Conv1d(128, 10000, 1)
#         self.fc1 = nn.Linear(10000, 512)
#         self.fc2 = nn.Linear(512, 256)
#         self.fc3 = nn.Linear(256, num_classes)
#         self.relu = nn.ReLU()
#         self.maxpool = nn.MaxPool1d(10000)
#         self.dropout = nn.Dropout(p=0.5)  # Aggiungi Dropout per prevenire l'overfitting

#     def forward(self, x):
#         x = self.relu(self.conv1(x))
#         x = self.dropout(x)  # Applicazione di dropout
#         x = self.relu(self.conv2(x))
#         x = self.dropout(x)  # Applicazione di dropout
#         x = self.relu(self.conv3(x))
#         x = self.maxpool(x)
#         x = x.view(-1, 10000)
#         x = self.relu(self.fc1(x))
#         x = self.dropout(x)  # Applicazione di dropout
#         x = self.relu(self.fc2(x))
#         x = self.fc3(x)
#         return x
    
# def get_pcd_file_paths(directory):
#     file_paths = []
#     for file_name in sorted(os.listdir(directory)):
#         if file_name.endswith('.pcd'):
#             file_paths.append(os.path.join(directory, file_name))
#     return file_paths

# def contains(substring, string):
#     return substring in string

# def predict(model, file_path, num_points=10000):
#     # Carica il file .pcd
#     pcd = o3d.io.read_point_cloud(file_path)
#     points = np.asarray(pcd.points)

#     # Campiona un numero fisso di punti
#     if points.shape[0] > num_points:
#         indices = np.random.choice(points.shape[0], num_points, replace=False)
#     else:
#         indices = np.random.choice(points.shape[0], num_points, replace=True)

#     sampled_points = points[indices]

#     # Converti in tensore e aggiungi una dimensione batch
#     point_cloud = torch.tensor(sampled_points, dtype=torch.float32).unsqueeze(0).transpose(2, 1)

#     # Fai la predizione
#     with torch.no_grad():
#         outputs = model(point_cloud)
#         _, predicted = torch.max(outputs.data, 1)

#     return predicted.item()

# # Esempio di utilizzo
# rospy.init_node("dataset", anonymous = True)
# file_paths = get_pcd_file_paths("/home/workstation2/ws_cross_modal/dataset")
# # file_paths = ["/home/workstation2/ws_cross_modal/dataset/cavo01.pcd", "/home/workstation2/ws_cross_modal/dataset/tavolo01.pcd"]
# labels = []
# for i in range(len(file_paths)):
#     if contains("cavo", file_paths[i]):
#         labels.append(1)
#     else:
#         labels.append(0)

# # print(file_paths)
# print(labels)
# num_points = 10000
# # print(point_cloud.shape)  # Dovrebbe stampare (N, 3) dove N è il numero di punti
# dataset = PointCloudDataset(file_paths, labels, num_points)
# dataloader = DataLoader(dataset, batch_size=1, shuffle=True)  # Modifica il batch_size in base alla memoria disponibile

# # Definizione del modello, criterio e ottimizzatore
# model = PointNet(num_classes=2)
# criterion = nn.CrossEntropyLoss()
# optimizer = optim.Adam(model.parameters(), lr=0.00001, weight_decay=1e-4)

# # Training loop con misurazione del tempo
# def train(model, dataloader, optimizer, criterion, num_epochs):
#     model.train()
#     start_time = time.time()
#     for epoch in range(num_epochs):
#         for point_clouds, labels in dataloader:
#             optimizer.zero_grad()
#             outputs = model(point_clouds)
#             loss = criterion(outputs, labels)
#             loss.backward()
#             optimizer.step()
#         print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}')
#     end_time = time.time()
#     print(f'Tempo totale di addestramento: {end_time - start_time:.2f} secondi')

# train(model, dataloader, optimizer, criterion, num_epochs=10)

# # Salvataggio del modello
# torch.save(model.state_dict(), "pointnet_model.pth")

import open3d as o3d
import numpy as np
import os
import torch
import torch.optim as optim
from sklearn.model_selection import train_test_split
from torch.utils.data import DataLoader, TensorDataset, WeightedRandomSampler

def search_max_points(pcd_dir):
    maximum = -1
    pcd_files = [f for f in os.listdir(pcd_dir) if f.endswith('.pcd')]
    for file in pcd_files:
        pcd_path = os.path.join(pcd_dir, file)
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points)
        if points.shape[0] > maximum:
            maximum = points.shape[0]
    return maximum

# Funzione per caricare i file PCD e estrarre i punti, campionando 1024 punti da ogni file
def load_pcd_files(pcd_dir, label, min_points):
    pcd_files = [f for f in os.listdir(pcd_dir) if f.endswith('.pcd')]
    data = []
    labels = []
    for file in pcd_files:
        pcd_path = os.path.join(pcd_dir, file)
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points)
        if points.shape[0] < min_points:
            # Pad with zeros if fewer points
            points = np.pad(points, ((0, min_points - points.shape[0]), (0, 0)), mode='constant', constant_values=0)
        # else:
        #     # Use all points or sample a subset
        #     points = points[np.random.choice(points.shape[0], min_points, replace=False)]
        data.append(points)
        labels.append(label)
    return data, labels

maximum_cable = search_max_points('/home/workstation2/ws_cross_modal/dataset/cable')
maximum_not_cable = search_max_points('/home/workstation2/ws_cross_modal/dataset/not_cable')

if maximum_cable <= maximum_not_cable:
    num_points = maximum_not_cable
else:
    num_points = maximum_cable

# Carica i dati da due directory, una per ciascuna classe
data_class_0, labels_class_0 = load_pcd_files('/home/workstation2/ws_cross_modal/dataset/not_cable', 0, num_points)
data_class_1, labels_class_1 = load_pcd_files('/home/workstation2/ws_cross_modal/dataset/cable', 1, num_points)

# Unisci i dati e le etichette
data = np.vstack([data_class_0, data_class_1])
labels = np.hstack([labels_class_0, labels_class_1])

# Converti in tensori
data = torch.tensor(data, dtype=torch.float32)
labels = torch.tensor(labels, dtype=torch.long)

# Bilancia il dataset per evitare squilibri tra le classi
class_sample_count = np.bincount(labels.numpy())
weights = 1. / class_sample_count
samples_weights = weights[labels]

# Crea un WeightedRandomSampler per bilanciare il dataset
sampler = WeightedRandomSampler(samples_weights, len(samples_weights))

# Suddividi in training e test set
data_train, data_test, labels_train, labels_test = train_test_split(data, labels, test_size=0.1, random_state=42)

# Converti in DataLoader
batch_size = 12
train_dataset = TensorDataset(data_train, labels_train)
test_dataset = TensorDataset(data_test, labels_test)
train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, drop_last=False)
test_loader = DataLoader(test_dataset, batch_size=batch_size, shuffle=False)

# Debugging: stampa le dimensioni dei dati caricati
print(f"Data shape: {data.shape}, Labels shape: {labels.shape}")
print(f"Train data shape: {data_train.shape}, Train labels shape: {labels_train.shape}")
print(f"Test data shape: {data_test.shape}, Test labels shape: {labels_test.shape}")

import torch.nn as nn
import torch.nn.functional as F

class PointNet(nn.Module):
    def __init__(self):
        super(PointNet, self).__init__()
        self.fc1 = nn.Linear(3, 64)
        self.fc2 = nn.Linear(64, 128)
        self.fc3 = nn.Linear(128, 256)
        self.fc4 = nn.Linear(256, 512)
        self.fc5 = nn.Linear(512, 256)
        self.fc6 = nn.Linear(256, 2)  # 2 per il problema di classificazione binaria

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))
        x = F.relu(self.fc5(x))
        x = torch.max(x, 1, keepdim=True)[0]  # Max pooling per ottenere una rappresentazione globale
        x = self.fc6(x)
        return x  # Restituisce un tensore di dimensioni [batch_size, num_classes]

def train_model(model, train_loader, criterion, optimizer, num_epochs):
    model.train()
    for epoch in range(num_epochs):
        running_loss = 0.0
        for data, labels in train_loader:
            # Debugging: stampa le dimensioni dei dati e delle etichette
            # print(f"Batch data shape: {data.shape}, Batch labels shape: {labels.shape}")
            optimizer.zero_grad()
            outputs = model(data).squeeze()  # Rimuove le dimensioni extra
            if outputs.dim() == 1:  # Assicura che outputs abbia la dimensione corretta
                outputs = outputs.unsqueeze(0)
            # print(f"Outputs shape: {outputs.shape}, Labels shape: {labels.shape}")  # Stampa per debugging
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            running_loss += loss.item()
        print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {running_loss/len(train_loader)}')

def test_model(model, test_loader):
    model.eval()
    correct = 0
    total = 0
    with torch.no_grad():
        for data, labels in test_loader:
            outputs = model(data).squeeze()  # Rimuove le dimensioni extra
            if outputs.dim() == 1:  # Assicura che outputs abbia la dimensione corretta
                outputs = outputs.unsqueeze(0)
            _, predicted = torch.max(outputs.data, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()
    print(f'Accuracy of the network on the test data: {100 * correct / total}%')

# Creazione dell'oggetto modello, della funzione di perdita e dell'ottimizzatore
# model = PointNet()
# criterion = nn.CrossEntropyLoss()
# optimizer = torch.optim.Adam(model.parameters(), lr=0.001)

# Addestramento del modello
# train_model(model, train_loader, criterion, optimizer, num_epochs=50)
# torch.save(model.state_dict(), "pointnet_model.pth")

# Test del modello
model_path = '/home/workstation2/ws_cross_modal/pointnet_model36.pth'
model = PointNet()
model.load_state_dict(torch.load(model_path))
test_model(model, test_loader)
