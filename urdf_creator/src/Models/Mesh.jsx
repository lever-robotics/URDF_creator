import * as THREE from "three";
export default class Mesh extends THREE.Mesh {
    constructor(urdfObject, shape, params) {
        /* DESCRIPTION:
            Mesh: This is the visual representation of the the link
            ------------------
            children: Mesh
                -> STL: I have a fat comment at the bottom of the class where you can prolly put logic stuff for the STL
        */
        super();
        this.isShape = true;
        this.geometry = defineGeometry(shape);
        this.material = new THREE.MeshPhongMaterial({
            color: Math.random() * 0xffffff,
        });
        this.customRenderBehaviors = { defineRenderBehavior: defineRenderBehavior(shape) };
        this.scale.set(...(params?.scale ?? [1, 1, 1]));

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
    onBeforeRender = () => {
        Object.values(this.customRenderBehaviors).forEach((behavior) => behavior(this));
    };

    /**
     * 
     * 
     * 
     * FAT comment for the STL logic
     * 
     * 
     * 
     * 
     * 
     */
}
