# AnimationCourse Final Project - Snake 3D By Itamar Ben-Atar && Ravid Rom

### The features we implemented

- 3D Snake model constructed from scaled snake object file that is divided into 16 joints.
    - Our model currently **supports texture**.  

- Supports a camera with 2 points of view: **Static (3rd person)** and **First person** (using Euler angles).

- **Interactive menu** at the beginning of the game and between levels, that indicates the **current score**, and supports the option to mute the background music.

- The **3D moving objects** that are featured in our game are:  
    - *"Normal"* spheres that add points to the player.
    - *"Super Speed"* cubes that increase the snake's speed and length for a certain amount of time.
    - *"Bomb"* cubes, that when interacted launch a missle attack of bouncing spheres that fall according to **gravity physics**, that if touched, end the game.
    - *"Game Over"* cubes, when touched they finish the game for the player.  
    
    Each 3D object is initialized with a **random velocity & direction**, and a random location inside the game.

- **Skinning** the 3D snake.

- In order to track interaction with 3D moving objects as mentioned above: 
    - For the spheres we calculated the position of the snake's head (absolute location) and subtracted that from each sphere's location, and if that location is smalled than the diameter that indicates there was an interaction.
    - For the cubes we used **collision detection** as implemented in assignment 2.

- When the the player starts the game he is prompted to choose a **difficulty level** to play in, normal or hard. During the game, the player completes each level by **scoring max points** and collecting all the "normal" spheres. The special objects do not add points. If the hard difficulty is selected the player has a set amount of time, which increses each level, to get all the points. If failed to collect all points on time, its game over. When the player completed a level he is promted with the option to exit or start the next level.

- Pressing 'p' on the keyboard will pause the game (snake, moving objects, and music, and if hard difficulty was chosen the countdown timer will pause as well).

- **Cube Map** - loaded a 3D cube map as an environemnt.

- **Sound:** 
    - Background Music - currently palying background music with the option to mute it through the interactive menu or by pressing 'm' on the kyeboard.
    - Interaction Sounds: when interacting with different 3D moving objects, the sound is **played in 3D**, meaning the sound is heard depending on the camera position and direction. 

### The difficulties we encountered during the development:
When we worked on the project we were optimistic and enthusiastic to create a cool game with a lot of cool features. The main setbacks we encountered during our progression to our goal was the lack of knowledge, online material and documentation on the technologies used in the engine: We are inexperinced with c++ and visual studio, we never used openGl, Eigen, libgil and other used libraries in the engine and libgil comes with low amount of online material and documentation. These hurdles shifted our time and focus from implementing the material we learned during the course and expanding our knowlege in the field of Computer Animation to learning new libraries languages.
