"""
Title: Deep Learning for Indoor Pedestal Fan Blade Inspection: Utilizing Low-Cost Autonomous Drones in an Educational Setting

Title for citation: Application for Drone Path-Planning and Deep Learning: Concept of Wind Turbine Inspection Using Drones

Authors:
    Angel Rodriguez (0009-0003-6360-3685)
    Joshua Zander (0009-0009-3965-8663)
    Mason Davis (0009-0003-5832-8738)
    Edwin Nazario Dejesus (0009-0009-2783-9547)
    Mohammad Shekaramiz (0000-0003-1176-3284)*
    Majid Memari (0000-0001-5654-4996)
    Mohammad A.S. Masoum (0000-0001-7513-313X)

Affiliation:
    Machine Learning and Drone Lab, Electrical and Computer Engineering Department, Utah Valley University, Orem, Utah, USA

Corresponding Author:
    Mohammad Shekaramiz: mshekaramiz@uvu.edu
"""


import torch
import torchvision.models as models
from transformers import ViTForImageClassification
from dataload import load_data, create_dataloader
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import confusion_matrix, precision_score, recall_score, f1_score, roc_curve, auc
from torch.optim.lr_scheduler import StepLR
import numpy as np

# Load the data and split into train, validate, and test sets
TRAINING_PATH = "/path/to/dataset"
(train_images, train_labels), (val_images, val_labels), (test_images, test_labels) = load_data(TRAINING_PATH, test_size=0.2, validation_size=0.1)
batch_size = 16
train_dataloader = create_dataloader(train_images, train_labels, batch_size)
val_dataloader = create_dataloader(val_images, val_labels, batch_size)
test_dataloader = create_dataloader(test_images, test_labels, batch_size)

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# Initialize the model based on the selected model name
def initialize_model(model_name):
    if model_name == 'alexnet':
        model = models.alexnet(weights=models.AlexNet_Weights.IMAGENET1K_V1)
        num_ftrs = model.classifier[6].in_features
        model.classifier[6] = torch.nn.Linear(num_ftrs, 2)
    elif model_name == 'vgg':
        model = models.vgg16(weights=models.VGG16_Weights.IMAGENET1K_V1)
        num_ftrs = model.classifier[6].in_features
        model.classifier[6] = torch.nn.Linear(num_ftrs, 2)
    elif model_name == 'xception':
        import pretrainedmodels
        model = pretrainedmodels.__dict__['xception'](num_classes=1000, pretrained='imagenet')
        num_ftrs = model.last_linear.in_features
        model.last_linear = torch.nn.Linear(num_ftrs, 2)
    elif model_name == 'resnet':
        model = models.resnet50(weights=models.ResNet50_Weights.IMAGENET1K_V1)
        num_ftrs = model.fc.in_features
        model.fc = torch.nn.Linear(num_ftrs, 2)
    elif model_name == 'densenet':
        model = models.densenet161(weights=models.DenseNet161_Weights.IMAGENET1K_V1)
        num_ftrs = model.classifier.in_features
        model.classifier = torch.nn.Linear(num_ftrs, 2)
    elif model_name == 'vit':
        model = ViTForImageClassification.from_pretrained("google/vit-base-patch16-224-in21k")
        model.classifier = torch.nn.Linear(model.config.hidden_size, 2)
    return model

# Choose the model (change model_name to switch models)
model_name = 'resnet'
model = initialize_model(model_name)
model.to(device)

# Hyperparameters for training
learning_rate = 1e-5
weight_decay = 1e-4
batch_size = 32
num_epochs = 20
patience = 2

# Define optimizer, loss function, and learning rate scheduler
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate, weight_decay=weight_decay)
criterion = torch.nn.CrossEntropyLoss()
scheduler = StepLR(optimizer, step_size=10, gamma=0.1)

# Training and validation loops
best_val_loss = float('inf')
early_stopping_counter = 0

for epoch in range(num_epochs):
    model.train()
    train_loss = 0.0
    train_correct = 0
    for images, labels in train_dataloader:
        images, labels = images.permute(0, 3, 1, 2).to(device), labels.to(device)  # Move data to device
        optimizer.zero_grad()
        outputs = model(images).logits if model_name == 'vit' else model(images)  # Forward pass
        loss = criterion(outputs, labels)  # Compute loss
        loss.backward()  # Backpropagation
        optimizer.step()  # Update weights
        train_loss += loss.item()
        train_correct += (outputs.argmax(1) == labels).sum().item()

    train_loss /= len(train_dataloader)
    train_acc = train_correct / len(train_dataloader.dataset)

    model.eval()
    val_loss = 0.0
    val_correct = 0
    with torch.no_grad():
        for images, labels in val_dataloader:
            images, labels = images.permute(0, 3, 1, 2).to(device), labels.to(device)
            outputs = model(images).logits if model_name == 'vit' else model(images)
            loss = criterion(outputs, labels)
            val_loss += loss.item()
            val_correct += (outputs.argmax(1) == labels).sum().item()

    val_loss /= len(val_dataloader)
    val_acc = val_correct / len(val_dataloader.dataset)

    # Check for early stopping
    if val_loss < best_val_loss:
        best_val_loss = val_loss
        early_stopping_counter = 0
    else:
        early_stopping_counter += 1
        if early_stopping_counter >= patience:
            break

    print(f'Epoch {epoch+1}/{num_epochs}, Train Loss: {train_loss:.4f}, Train Acc: {train_acc:.4f}, Val Loss: {val_loss:.4f}, Val Acc: {val_acc:.4f}')
    scheduler.step()

# Testing the model
model.eval()
test_loss = 0.0
test_correct = 0
all_preds = []
all_labels = []
all_probs = []

with torch.no_grad():
    for images, labels in test_dataloader:
        images, labels = images.permute(0, 3, 1, 2).to(device), labels.to(device)
        outputs = model(images).logits if model_name == 'vit' else model(images)
        loss = criterion(outputs, labels)
        test_loss += loss.item()
        test_correct += (outputs.argmax(1) == labels).sum().item()
        all_preds.extend(outputs.argmax(1).cpu().numpy())
        all_labels.extend(labels.cpu().numpy())
        all_probs.extend(torch.nn.functional.softmax(outputs, dim=1).cpu().numpy())

test_loss /= len(test_dataloader)
test_acc = test_correct / len(test_dataloader.dataset)
precision = precision_score(all_labels, all_preds)
recall = recall_score(all_labels, all_preds)
f1 = f1_score(all_labels, all_preds)
print(f'Test Loss: {test_loss:.4f}, Test Acc: {test_acc:.4f}, Precision: {precision:.4f}, Recall: {recall:.4f}, F1: {f1:.4f}')

# Visualize results
def plot_confusion_matrix(labels, predictions):
    cm = confusion_matrix(labels, predictions)
    sns.heatmap(cm, annot=True, cmap='Blues', fmt='g')
    plt.xlabel('Predicted')
    plt.ylabel('True')
    plt.title('Confusion Matrix')
    plt.savefig("confusion_matrix.png")

def plot_roc_curve(labels, probs):
    fpr, tpr, _ = roc_curve(labels, probs[:, 1])
    roc_auc = auc(fpr, tpr)
    plt.plot(fpr, tpr, label=f'ROC curve (area = {roc_auc:.2f})')
    plt.xlabel('False Positive Rate')
    plt.ylabel('True Positive Rate')
    plt.title('ROC Curve')
    plt.legend()
    plt.savefig("roc_curve.png")

plot_confusion_matrix(all_labels, all_preds)
plot_roc_curve(all_labels, np.array(all_probs))
