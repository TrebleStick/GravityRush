
# Gravity Rush

We present a mobile app solution for the rehabilitation of muscles following injury. Gravity Rush is an arcade style game controlled by two surface electromyography (sEMG) electrodes placed on the target muscle, the sensors are connected to a mobile device via Bluetooth and provide data for a convolutional neural network that classifies the data into two different poses. These poses are used as inputs to control the game mechanics, which are based on navigating a rocket through space avoiding asteroids and black holes. Once a player hits a black hole the game ends and their score is recorded in a leaderboard, it is possible for players to build a friends list adding to the competitiveness of the game.

## app/
###### Gravity Rush
XCode workspace for the finished IOS application.

###### img
In game assest.

###### Muscle
XCode workspace for control app for user testing.

###### Python
Scripts for random generation of in game assets.

## Docs/
Misc storage.

## Hardware
PCB firmware work environments.

## ML/
Data collection files.

###### AE_model
Workspace for editing CNN model.

###### EMG
Arduino code for data collection.

###### EMG_data
Data storage for training CNN.


# Notes for Contributors
### Git LF usage

Use git LFS when adding larger files such as build apps, firmware project files etc.

See instructions here : https://git-lfs.github.com/
