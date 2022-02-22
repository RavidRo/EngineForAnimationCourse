# EngineForAnimationCourse Final Project

Snake 3D By Itamar Ben-Atar && Ravid Rom

- 3D Snake model constructed from scaled snake object file that is divided into 16 joints.
    - Our model currently **supports texture**.  

- Supports a camera with 2 points of view: **Static (3rd person)** and **First person** (using Euler angles).

- **Interactive menu** at the beginning of the game and between levels, that indicates the **current score**, and supports the option to mute the background music.

- The **3D moving objects** that are featured in our game are:  
    - "Normal" spheres that add points to the player.
    - "Super Speed" cubes that increase the snake's speed and length for a certain amount of time.
    - "Bomb" cubes, that when interacted launch a missle attack of bouncing spheres that fall according to **gravity physics**, that if touched, end the game.
    - "Game Over" cubes, when touched they finish the game for the player.  
    
    Each 3D object is initialized with a **random velocity & direction**, and a random location inside the game.

- **Skinning** the 3D snake.

- In order to track interaction with 3D moving objects as mentioned above: 
    - For the spheres we calculated the position of the snake's head (absolute location) and subtracted that from each sphere's location, and if that location is smalled than the diameter that indicates there was an interaction.
    - For the cubes we used **collision detection** as implemented in assignment 2.

-Each level comes to an end when the player has scored the max points for that level - meaning he has collected all the "normal" spheres, the special feature objects do not add points to the player.
After reaching the max score for that level the player has an option wether to exit or start the next level.

- **Cube Map** - loaded a 3D cube map as an environemnt.

- **Sound:** 
    - Background Music - currently palying background music with the option to mute it through the interactive menu or by pressing 'm' on the kyeboard.
    - Interaction Sounds: when interacting with different 3D moving objects, the sound is **played in 3D**, meaning the sound is heard depending on the camera position and direction. 

The difficulties we encountered during our  