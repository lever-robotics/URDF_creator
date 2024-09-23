# Tree Structure

## Diagram
[![](https://mermaid.ink/img/pako:eNqdUsFOwzAM_RUrJyZ14p4DEgKBkBgghjjlErVua9baVZICY-zfSZtuGkO7cEnil2e_Z8sblUuBSqu8sd5fk62cbQ3fxBMXlm2FDuZzGGPD00dEvi_gTYjDK_neNvSFzvARkFh5TU3hMCX6E6R74pXh4UzxHaMLZA-hlHKILJG9uEPkSpqGPMlvn5ef5AfrO1DDo6OKOINnCTaM9GNXGl6w7VKLOxacw5N4SgmjaKxUlh4DnH1QqMGh7zAPECTNa2Z40NZ7nZO0qd9Yj5s1CCN00cSgkcGD8PvUe-pYgx9vKGyII0qu9eANLBeDGMho64_cUHCWwS1Ki8GtDe8H9r90wypTLbrWUhFXaGMYwKhQY9wVpeOzsG5llOFt5Nk-yHLNudLB9ZgpJ31VK13axseo72I3OO3fHsWCgrjFtKHCJVVq-wOLt_aG?type=png)](https://mermaid.live/edit#pako:eNqdUsFOwzAM_RUrJyZ14p4DEgKBkBgghjjlErVua9baVZICY-zfSZtuGkO7cEnil2e_Z8sblUuBSqu8sd5fk62cbQ3fxBMXlm2FDuZzGGPD00dEvi_gTYjDK_neNvSFzvARkFh5TU3hMCX6E6R74pXh4UzxHaMLZA-hlHKILJG9uEPkSpqGPMlvn5ef5AfrO1DDo6OKOINnCTaM9GNXGl6w7VKLOxacw5N4SgmjaKxUlh4DnH1QqMGh7zAPECTNa2Z40NZ7nZO0qd9Yj5s1CCN00cSgkcGD8PvUe-pYgx9vKGyII0qu9eANLBeDGMho64_cUHCWwS1Ki8GtDe8H9r90wypTLbrWUhFXaGMYwKhQY9wVpeOzsG5llOFt5Nk-yHLNudLB9ZgpJ31VK13axseo72I3OO3fHsWCgrjFtKHCJVVq-wOLt_aG)

## Explanation

### FrameManger

    I was reading this website [url](https://refactoring.guru/) and decided to implement a manager pattern. It made sense for how complicated the urdfObject was getting and helped abstract some details away from it. Just give it a peruse it is pretty self explanatory. It has no relation to the react or the scene, it's only purpose it to make or modify urdfObjects and their tree structure. I think it will help make it easier to add/remove/modify the tree structure going forward.

### Frame
    So I changed the tree structure but don't worry it makes way more sense. It is super easy to understand now. I didn't have time to go through the scene to xml/sdf/text and update them properly so I'm pretty sure I broke them ;( sorry mark. But they should be easy to fix and way more intuitive.

    Also I deleted shimmy but we can totally still implement a shimmy function. In fact right now if you mess with a joint and move it up/down or rotate it, after releasing the slider it returns to normal. We can modify this functionality to make a shimmy!

    The overall black box. It stores the Position and Rotation data of the whole tree object.
    
    There are getter/setter functions in this object. Basically anything that needs to reference anything else in the strucutre I have either already made a get/set for or you can easily make one. This lets the rest of the program access anything in the tree by only needing to call urdfObject.getFoo

    There are logic functions that made sense to include in the object as well. Helped simplify sceneState and reduce redundant/unecessary calls to the object

### JointVisualizer
    Currently Joint displays the joint axis with its rotation and position data

### Link
    Contains the offset data. We used to mathmatially reverse the amount the joint moved in any direction to get the links offset but this didn't work so well if a joint got moved and rotated. Now whenever a joint is moved or rotated I just detach the link from the joint, move it, then reattach it. So it's position data will hold it's relative difference to the joint from which offset can easily be obtained.

### Mesh
    This is the actual visual aspect of the urdfObject. It also holds the scaling data. I believe you will add the STL as a direct Child of this mesh. Then as the mesh gets scaled it can scale the STL. We might need to finangle scaling a bit if we don't want the STL to scale proportional to the mesh but one step at a time haha. I also modified scaling a bit. Because of the way we were using a uniform scaling equation it made user input inaccurate. If the user inputed a radius of 8 in the parameter drop down it would get uniformed and changed to something different. I felt like that would be annonying to users and implemented a different approach. It is a little messy because I had to user defineProperty to override the original THREE.js scale property and I had to build a custom Vector3 class that could ensure a sphere would have the same scale value along every axis. I read A LOT of the three.js code on github and this was the best solution I could bake up.

### childrenUrdfObjects
    children urdfObjects are actually children of urdfObjects. Crazy!

### UserData
    I realized that all the data held in user data was also being stored within the tree structure. It didn't make sense to keep track of the same information twice. So I got rid of user data and instead tried to make all the data easily accesible and updatable.

    Now when converting the three scene to json, gltf, xml, urdf or anything we would normally place all this extra data in the userData section for every link. I plan to make a little function that grabs all necessary data from the tree structure and presents it in a json like object. This should make everything user data related easer to access, store, convert, and update. 