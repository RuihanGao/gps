import torch
import torch.nn as nn
import torch.nn.functional as F 

class Net(nn.Module):
    def __init__(self, state_size=(48, 48, 1), action_size=None):
        # TODO: figure out the action_size
        # TODO: what's the second channel? Need actions since the list is not discrete?
        super(Net, self).__init__()
        shape = state_size[0:2]

        # kernel formula new_size = (W-F+2*P)/S+1
        # out_channels?
        self.conv1 = nn.Conv2d(state_size[-1], out_channels=64, kernel_size=3, stride=2)
        shape = [(shape[0]-3)//2+1, (shape[1]-3)//2+1] #23
        self.conv2 = nn.Conv2d(64, out_channels=32, kernel_size=3, stride=2)
        shape = [(shape[0]-3)//2+1, (shape[1]-3)//2+1] #11
        self.conv3 = torch.nn.Conv2d(32, 32, kernel_size=3, stride=1)
        shape = [(shape[0]-3)//2+1, (shape[1]-3)//2+1] #5
        shape = shape[0]*shape[1]*32 #5*5*16=400
        # an affine operation: y = Wx+b
        
        # TODO: number of features?
        self.fc1 = nn.Linear(64*3*3, 32*3*3)
        self.fc2 = nn.Linear(32*3*3, 32*3*3)
        self.fc3 = nn.Linear(32*3*3, action_size)
       
    
    def forward(self, x):
        """ 
        Build a policy network that maps states to actions.
        """
        # # Max pooling over a (2, 2) window
        # x = F.max_pool2d(F.relu(self.conv1(x)), (2, 2))
        # x = x.view(-1, self.num_flat_features(x))
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        # scale the continuous action within [-1, 1], and use tanh activation function
        # similar to BipedalWalker example,
        # ref: https://github.com/openai/gym/wiki/BipedalWalker-v2
        x = torch.tanh(self.fc3(x))  
        return x 

    
    def num_flat_features(self, x):
        size = x.size()[1:]  # all dimensions except the batch dimension
        num_features = 1 
        for s in size:
            num_features *= s
        return num_features

net = Net()
print(net)

# check learnable parameters
params = list(net.parameters())
print(len(params))
print(params[0].size())


# loss funtion
output = net(input)
target = torch.randn(10)
target = target.view(1, -1)
criterion = nn.MSELoss()
loss = criterion(output, target)
print(loss)
print(loss.grad_fn)  # MSELoss
print(loss.grad_fn.next_functions[0][0])  # Linear
print(loss.grad_fn.next_functions[0][0].next_functions[0][0])  #ReLU

net.zero_grad()  # zeros the gradient buffers of all parameters
print("conv1.bias.grad before backward")
print(net.conv1.bias.grad)
loss.backward()
print("conv1.bias.grad after backward")
print(net.conv1.bias.grad)

# update the weights
learning_rate = 0.01
for f in net.parameters():
    f.data.sub_(f.grad.data*learning_rate)

import torch.optim as optim

# create your optimizer
optimizer = optim.SGD(net.parameters(), lr = 0.01)
# in your trianing loop
optimizer.zero_grad()
output = net(input)
loss = criterion(output, target)
loss.backward()
optimizer.step()



