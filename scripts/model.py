import open3d as o3d
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import os

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

# Definizione della classe del modello (deve essere uguale a quella usata durante l'addestramento)
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

def load_pcd_files(pcd_dir):
    pcd_files = [f for f in os.listdir(pcd_dir) if f.endswith('.pcd')]
    paths = []
    for file in pcd_files:
        pcd_path = os.path.join(pcd_dir, file)
        paths.append(pcd_path)
    return paths

# Funzione per caricare e pre-processare una nuova point cloud
def load_and_preprocess_pcd(pcd_path, num_points):
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    if points.shape[0] < num_points:
        points = np.pad(points, ((0, num_points - points.shape[0]), (0, 0)), mode='constant', constant_values=0)
    # else:
    #     points = points[np.random.choice(points.shape[0], num_points, replace=False)]
    return torch.tensor(points, dtype=torch.float32)

maximum_cable = search_max_points('/home/workstation2/ws_cross_modal/dataset/cable')
maximum_not_cable = search_max_points('/home/workstation2/ws_cross_modal/dataset/not_cable')

if maximum_cable <= maximum_not_cable:
    num_points = maximum_not_cable
else:
    num_points = maximum_cable

# Percorso del file del modello addestrato
model_path = "pointnet_model_augmented.pth"

# Caricamento del modello
model = PointNet()
model.load_state_dict(torch.load(model_path))
model.eval()

# Percorso del nuovo file PCD
path_not_cable = '/home/workstation2/ws_cross_modal/dataset/not_cable/'
path_cable = '/home/workstation2/ws_cross_modal/dataset/cable/'

# for i in range(21):
# new_pcd_path = path_not_cable + str((29)) + '.pcd'
# print(new_pcd_path)

paths = load_pcd_files(path_not_cable)

# Caricamento e pre-processamento della nuova point cloud
for file in range(len(paths)):
    new_points = load_and_preprocess_pcd(paths[file], num_points)

    # Aggiunge una dimensione batch
    new_points = new_points.unsqueeze(0)  # Dimensione [1, num_points, 3]

    # Previsione
    with torch.no_grad():
        outputs = model(new_points).squeeze()
        _, predicted = torch.max(outputs.data, 0)
        predicted_class = predicted.item()

    print(f'Predicted class: {predicted_class}')
