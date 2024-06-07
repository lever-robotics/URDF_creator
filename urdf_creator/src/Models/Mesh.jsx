import * as THREE from "three";

export default class Mesh extends THREE.Mesh {
    constructor(shape, params) {
        /* DESCRIPTION:
            Mesh: This is the visual representation of the the link
            ------------------
            properties: The desired properties
                -> Add the desired name as the key and the value to be set as the value
            children: Mesh should'nt have any children
                -> 
            attributes: Mesh contains the scale attribute
                -> Add attributes to the attribute object. Then set them to the object
        */
        super();
        //***Properties-Children-Attributes***/
        const properties = {
            isShape: true,
            geometry: defineGeometry(shape),
            material: new THREE.MeshPhongMaterial({
                color: Math.random() * 0xffffff,
            }),
            customRenderBehaviors: { "defineRenderBehavior": defineRenderBehavior(shape) }
        };
        const children = {};
        const attributes = {
            scale: params?.scale ?? [1, 1, 1]
        };

        //***Assign-add()-set()***/
        const assignProperties = (elements) => {
            // These are automatic
            Object.entries(elements).forEach(([key, value]) => {
                this[key] = value;
            });
        };
        assignProperties(properties);
        assignProperties(children);
        // Add scene here
        // Set attributes here
        this.scale.set(...attributes.scale);

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
                    return new THREE.CylinderGeometry(0.5, 0.5, 1, 32);
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
                        const uniformScale =
                            (worldScale.x + worldScale.y + worldScale.z) / 3;

                        const localScale = context.scale;
                        context.scale.set(
                            (localScale.x / worldScale.x) * uniformScale,
                            (localScale.y / worldScale.y) * uniformScale,
                            (localScale.z / worldScale.z) * uniformScale
                        );
                    };
                case "cylinder":
                    // ensure cylinders scale uniformly in two directions
                    return (context) => {
                        const worldScale = new THREE.Vector3();
                        context.getWorldScale(worldScale);
                        const uniformScale = (worldScale.x + worldScale.z) / 2;

                        const localScale = context.scale;
                        context.scale.set(
                            (localScale.x / worldScale.x) * uniformScale,
                            localScale.y,
                            (localScale.z / worldScale.z) * uniformScale
                        );
                    }
                default:
                    return;
            }
        }
    }
    onBeforeRender = () => {
        Object.values(this.customRenderBehaviors).forEach((behavior) => behavior(this));
    };
}