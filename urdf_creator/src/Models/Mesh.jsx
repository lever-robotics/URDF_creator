import * as THREE from "three";

export default class Mesh extends THREE.Mesh {
    constructor(shape, params) {
        let geometry;
        let customRenderBehavior = () => {};
        switch (shape) {
            case "cube":
                geometry = new THREE.BoxGeometry(1, 1, 1);
                break;
            case "sphere":
                geometry = new THREE.SphereGeometry(0.5, 32, 32);
                // ensure spheres scale uniformly in all directions
                customRenderBehavior = function (
                    renderer,
                    scene,
                    camera,
                    geometry,
                    material,
                    group
                ) {
                    const worldScale = new THREE.Vector3();
                    this.getWorldScale(worldScale);
                    const uniformScale =
                        (worldScale.x + worldScale.y + worldScale.z) / 3;

                    const localScale = this.scale;
                    this.scale.set(
                        (localScale.x / worldScale.x) * uniformScale,
                        (localScale.y / worldScale.y) * uniformScale,
                        (localScale.z / worldScale.z) * uniformScale
                    );
                };
                break;
            case "cylinder":
                geometry = new THREE.CylinderGeometry(0.5, 0.5, 1, 32);
                // ensure cylinders scale uniformly in two directions
                customRenderBehavior = function (
                    renderer,
                    scene,
                    camera,
                    geometry,
                    material,
                    group
                ) {
                    const worldScale = new THREE.Vector3();
                    this.getWorldScale(worldScale);
                    const uniformScale = (worldScale.x + worldScale.z) / 2;

                    const localScale = this.scale;
                    this.scale.set(
                        (localScale.x / worldScale.x) * uniformScale,
                        localScale.y,
                        (localScale.z / worldScale.z) * uniformScale
                    );
                };
                break;
            default:
                return;
        }

        const material = new THREE.MeshPhongMaterial({
            color: Math.random() * 0xffffff,
        });

        super(geometry, material);
        this.customRenderBehavior = customRenderBehavior;
        this.isShape = true;
    }
    onBeforeRender = (renderer, scene, camera, geometry, material, group) => {
        // this.uniformScaler.doScale();
        this.customRenderBehavior(
            renderer,
            scene,
            camera,
            geometry,
            material,
            group
        );
    };

    addUniformScaler = (scaler) => {
        super.add(scaler);
        this.uniformScaler = scaler;
    };

    add = (property) => {
        const className = property.constructor.name;
        console.log(className);
        if (className === "SceneObject") {
            this.setJoint(property);
        } else if (className === "UniformScaler") {
            this.addUniformScaler(property);
        }
    };

    setJoint = (joint) => {
        this.joint = joint;
    };
}

//     //***Properties-Children-Attributes***/
//     const properties = {};
//     const children = {};
//     const attributes = {
//         position: params?.position ?? [0, 0, 0],
//         rotation: params?.rotation ?? [0, 0, 0],
//     };

//     const defineGeometry = (shape, a, b, c) => {
//         switch (shape) {
//             case "cube":
//                 // a = width
//                 // b = height
//                 // c = depth
//                 return new THREE.BoxGeometry(a, b, c);
//             case "sphere":
//                 // a = radius
//                 // b = widthSegments
//                 // c = heightSegments
//                 return new THREE.SphereGeometry(a, b, c);
//             case "cylinder":
//                 // a = top and bottom radius
//                 // b = height
//                 // c = radialSegments = 'number of segmented faces around circumference
//                 return new THREE.CylinderGeometry(a, a, b, c);
//             default:
//                 console.log('No shape provided to defineGeometry');
//                 return;
//         }
//     }
//     const defineRenderBehavior = (shape) => {
//         switch (shape) {
//             case "cube":
//                 return;
//             case "sphere":
//                 // ensure spheres scale uniformly in all directions
//                 customRenderBehavior = function (
//                     renderer,
//                     scene,
//                     camera,
//                     geometry,
//                     material,
//                     group
//                 ) {
//                     const worldScale = new THREE.Vector3();
//                     this.getWorldScale(worldScale);
//                     const uniformScale =
//                         (worldScale.x + worldScale.y + worldScale.z) / 3;

//                     const localScale = this.scale;
//                     this.scale.set(
//                         (localScale.x / worldScale.x) * uniformScale,
//                         (localScale.y / worldScale.y) * uniformScale,
//                         (localScale.z / worldScale.z) * uniformScale
//                     );
//                 };
//                 return customRenderBehavior;
//             case "cylinder":
//                 // ensure cylinders scale uniformly in two directions
//                 customRenderBehavior = function (
//                     renderer,
//                     scene,
//                     camera,
//                     geometry,
//                     material,
//                     group
//                 ) {
//                     const worldScale = new THREE.Vector3();
//                     this.getWorldScale(worldScale);
//                     const uniformScale = (worldScale.x + worldScale.z) / 2;

//                     const localScale = this.scale;
//                     this.scale.set(
//                         (localScale.x / worldScale.x) * uniformScale,
//                         localScale.y,
//                         (localScale.z / worldScale.z) * uniformScale
//                     );
//                 };
//                 return customRenderBehavior;
//             default:
//                 console.log('No shape provided to customRenderBehavior');
//                 return;
//         }

//     }
//     const customRenderBehavior = defineRenderBehavior(shape);
//     const geometry = defineGeometry(shape, a, b, c);

//     const material = new THREE.MeshPhongMaterial({
//         color: Math.random() * 0xffffff,
//     });

//     super(geometry, material);
//     this.customRenderBehavior = customRenderBehavior;
//     this.isShape = true;
// }

// onBeforeRender = (renderer, scene, camera, geometry, material, group) => {
//     // this.uniformScaler.doScale();
//     this.customRenderBehavior(
//         renderer,
//         scene,
//         camera,
//         geometry,
//         material,
//         group
//     );
// };

// add = (property) => {
//     const className = property.constructor.name;
//     console.log(className);
//     if (className === "SceneObject") {
//         this.setJoint(property);
//     } else if (className === "UniformScaler") {
//         this.addUniformScaler(property);
//     }
// };

// super();
/* DESCRIPTION:
urdfObject: The encompassing object. Contains children/grandchildren that makeup the Joint/Link logic of a URDF
------------------
properties: Properties of the urdfObject are the values/references to pertinent flags and user defined data.
    -> To modify the properties of urdfObject simply add the property name as the object key and the value as the value. A function later in the constructor will auto add these values to the object. 
children: Direct children of urdfObject.
    -> Add direct children of urdfObject here. Their references will be automatically assigned as properties to the urdfObject. REMEMBER to use the add() function to add the references to the THREE.Object3D also
attributes: These are the values that THREE function will directly modify to change the state of the scene.
    -> Add all attributes and their default values here and set them corresespondingly below
*/

//***Properties-Children-Attributes***/
// const properties = {
//     urdfObject: true, // Flag to determine if is urdfObject
//     userData: new UserData(shape, name),
// };
// const children = {
//     joint: new Joint(params),
//     shimmy: new Shimmy(shape, params),
// };
// const attributes = {
//     position: params?.position ?? [0, 0, 0],
//     rotation: params?.rotation ?? [0, 0, 0],
// };

// //***Assign-add()-set()***//
// const assignProperties = (elements) => {
//     // These are automatic
//     Object.entries(elements).forEach(([key, value]) => {
//         this[key] = value;
//     });
// };
// assignProperties(properties);
// assignProperties(children);
// // Add Children here...
// this.add(this.joint);
// this.add(this.shimmy);
// // Add direct attributes here...
// this.position.set(...attributes.position);
// this.rotation.set(...attributes.rotation);
