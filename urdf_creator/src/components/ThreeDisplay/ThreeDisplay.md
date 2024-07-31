# ThreeDisplay

## Diagram


## Explanation

### ThreeDisplay

    I was having troubles modifing some THREE stuff with react and decided to abstract the THREE scene up one more layer. So now the THREE stuff is the parent object to SceneState. kinda like having the THREE in the background and the React in the foreground. I think it may increase performance too because react won't rerender the THREE stuff everytime SceneState get updated.

### ThreeSceneObject
    Basically everything in the InitScene file and the threeObjects useRef got moved to this class. Now it is an object and assigned as a useRef in ThreeDisplay. Should make it a little more readable and also easier to change/modify. Because it's a little beefy I made a manager for it. Honestly I am undecided if the manager made the code more or less readable... So we may decide to get rid of the manager. Either way I do like having the ThreeSceneObject.

### ThreeSceneObjectManager
    Manages the ThreeSceneObject

### Mouse
    I moved everything in the setupMouse file to this mouse object.

### SceneState
    This guy didn't change a whole ton. It is still the bridge between react and three. I tried to enforce a communication protocol. If any children of SceneState wanted to modify the scene or the selectedObject/urdfObject they had to do it through sceneState. However any children of SceneState could read data directly from the selectedObject. This I think helps with the unidirectional dataflow? I tried to start removing state from a lot of the child components of sceneState becuase most every child component eventually calls a function in SceneState that triggers a reRender. So be eliminating as much state from the children as possible we avoid the children double or quatruple rendering. 

