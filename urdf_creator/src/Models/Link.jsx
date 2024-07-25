import * as THREE from "three";

export default class Link extends THREE.Mesh {
    constructor(offset, shape, scale) {
        super();
        /* DESCRIPTION:
        Link: Used to demonstrate joint logic and keep data on specific joint keyframes
        ------------------
        properties: Properties of Link are the values/references to pertinent flags and user defined data.
            -> To modify the properties of Link simply add the property name as the object key and the value as the value. A function later in the constructor will auto add these values to the object. 
        children: Direct children of Link.
            -> Add direct children of Link here. Their references will be automatically assigned as properties to the Link. REMEMBER to use the add() function to add the references to the THREE.Object3D also
        attributes: These are the values that THREE function will directly modify to change the state of the scene.
            -> Add all attributes and their default values here and set them corresespondingly below
        */
        // this.mesh = new Mesh(shape, params);
            // position: params?.position ?? [0, 0, 0]

        // Add Children here...
        // this.add(this.mesh);
        // Set attributes here
        this.position.set(...offset);// The offset from the joint
        // this.position.set(...attributes.position); // The offset from the joint
        this.isShape = true;
        this.geometry = defineGeometry(shape);
        this.material = new THREE.MeshPhongMaterial({
            color: Math.random() * 0xffffff,
        });
        this.customRenderBehaviors = { defineRenderBehavior: defineRenderBehavior(shape) };
        // this.scale.set(...(params?.scale ?? [1, 1, 1]));
        this.scale.set(...scale);
        //***Helper Function***/
        function defineGeometry(shape, a, b, c) {
            switch (shape) {
                case "cube":
                    // a = width
                    // b = height
                    // c = depth
                    return new THREE.BoxGeometry(1, 1, 1);
                case "sphere":
                    // a = radius
                    // b = widthSegments
                    // c = heightSegments
                    return new THREE.SphereGeometry(0.5, 32, 32);
                case "cylinder":
                    // a = top and bottom radius
                    // b = height
                    // c = radialSegments = 'number of segmented faces around circumference
                    const cylinder = new THREE.CylinderGeometry(0.5, 0.5, 1, 32);
                    cylinder.rotateX(Math.PI / 2);
                    return cylinder;
                default:
                    return;
            }
        }
        function defineRenderBehavior(shape) {
            switch (shape) {
                case "cube":
                    return () => { };
                case "sphere":
                    // ensure spheres scale uniformly in all directions
                    return (context) => {
                        const worldScale = new THREE.Vector3();
                        context.getWorldScale(worldScale);
                        const uniformScale = (worldScale.x + worldScale.y + worldScale.z) / 3;

                        const localScale = context.scale;
                        context.scale.set((localScale.x / worldScale.x) * uniformScale, (localScale.y / worldScale.y) * uniformScale, (localScale.z / worldScale.z) * uniformScale);
                    };
                case "cylinder":
                    // ensure cylinders scale uniformly in two directions
                    return (context) => {
                        const worldScale = new THREE.Vector3();
                        context.getWorldScale(worldScale);

                        // the absolute values prevent an error that will cause the cylinder to disappear
                        // when the worldscale.z goes negative, it also flips the x scale to negative to prevent the cylinder from flipping horizontally
                        // but this means that worldScale.x + worldScale.y = 0 :(
                        // so making them abs will solve this
                        const uniformScale = (Math.abs(worldScale.x) + Math.abs(worldScale.y)) / 2;
                        const localScale = context.scale;
                        context.scale.setX((localScale.x / worldScale.x) * uniformScale);
                        context.scale.setY((localScale.y / worldScale.y) * uniformScale)
                    };
                default:
                    return;
            }
        }
    }

    set offset (offset) {
        this.position.set(...offset);
    }

    getOffset = () => {
        return this.position;
    } 

    addOffset (offset) {
        this.position.add(offset);
    }
    onBeforeRender = () => {
        Object.values(this.customRenderBehaviors).forEach((behavior) => behavior(this));
    };
}