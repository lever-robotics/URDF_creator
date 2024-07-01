# Tree Structure

## Diagram
[![](https://mermaid.ink/img/pako:eNplkb1uwzAMhF9F4Oygu4ZOWVqkCFAnmxZWom0mFlXoZwjSvHvl2AnSZuR3R_IoncEGR6DBjpjSmrGP6I2U6Lrt14FsVqvVz6t6Dyz5GbcDe3965vtEcY0ZjcyOmW5YjkZu2szehGJm_I9bkhSikallJh-UhsfaDjy6SLK_r05GJtMyYLcx8ieYVtvIPUujPkPGzKHK17Oqcs-p1WFCN8dLO7Jj6R_iTeZrijqv6xLlZatWrcWxeqEBT9Eju_qqZyNKGcgDeTJQe8FhPBowcqk-LDm0J7GgcyzUQAylH0B3OKZalW-HmZYvWejlF1kqlv4?type=png)](https://mermaid.live/edit#pako:eNplkb1uwzAMhF9F4Oygu4ZOWVqkCFAnmxZWom0mFlXoZwjSvHvl2AnSZuR3R_IoncEGR6DBjpjSmrGP6I2U6Lrt14FsVqvVz6t6Dyz5GbcDe3965vtEcY0ZjcyOmW5YjkZu2szehGJm_I9bkhSikallJh-UhsfaDjy6SLK_r05GJtMyYLcx8ieYVtvIPUujPkPGzKHK17Oqcs-p1WFCN8dLO7Jj6R_iTeZrijqv6xLlZatWrcWxeqEBT9Eju_qqZyNKGcgDeTJQe8FhPBowcqk-LDm0J7GgcyzUQAylH0B3OKZalW-HmZYvWejlF1kqlv4)

## Explanation

### urdfObject
    The overall black box. It stores the Position and Rotation data of the whole tree object.
    
    There are getter/setter functions in this object. Basically anything that needs to reference anything else in the strucutre I have either already made a get/set for or you can easily make one. This lets the rest of the program access anything in the tree by only needing to call urdfObject.getFoo

    There are logic functions that made sense to include in the object as well. Helped simplify sceneState and reduce redundant/unecessary calls to the object

### Joint
    Currently Joint displays the joint line but doesn't directly contain any important data. It will become useful when we implement the shimmy feature

### Shimmy
    Holds the joint Rotation and sliding data

### UserData
    Slowly setting up the urdfObject to continually update UserData so data can be accessed easier AND it will make converting between file types easier. Like when we save to a STL/GLTF.

### Link
    Contain the offset data. So if we move a joint 3 inches to the left then the offset value would bring the visual mesh back to it's original location. This lets us move the joint axis around while keeping the visual mesh in the same spot. Also the urdf needs the offset data.

### Mesh
    This is the actual visual aspect of the urdfObject. It also holds the scaling data. I believe you will add the STL as a direct Child of this mesh. Then as the mesh gets scaled it can scale the STL. We might need to finangle scaling a bit if we don't want the STL to scale proportional to the mesh but one step at a time haha. 

### childrenUrdfObjects
    children urdfObjects are placed as children of the Link

