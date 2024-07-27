import * as THREE from "three";

export default class ScaleVector extends THREE.Vector3 {
    constructor(shape, x, y, z){
        super(x, y, z);

        this.shape = shape;
    }

    copy(vector){
        switch (this.shape) {
            case "cube":
                super.copy(vector);
                break;
            case "sphere":
                // ensure spheres scale uniformly in all directions
                // return (context) => {
                //     const worldScale = new THREE.Vector3();
                //     context.getWorldScale(worldScale);
                //     const uniformScale =
                //         (worldScale.x + worldScale.y + worldScale.z) / 3;

                //     const localScale = context.scale;
                //     context.scale.set(
                //         (localScale.x / worldScale.x) * uniformScale,
                //         (localScale.y / worldScale.y) * uniformScale,
                //         (localScale.z / worldScale.z) * uniformScale
                //     );
                // };
                const x = vector.getComponent(0);
                const y = vector.getComponent(1);
                const z = vector.getComponent(2);

                if(x !== this.getComponent(0)){
                    super.copy(new THREE.Vector3(x,x,x));
                }else if(y !== this.getComponent(1)){
                    super.copy(new THREE.Vector3(y,y,y));
                }else if(z !== this.getComponent(2)){
                    super.copy(new THREE.Vector3(z,z,z));
                }
                break;
            case "cylinder":
                // ensure cylinders scale uniformly in two directions
                // return (context) => {
                //     const worldScale = new THREE.Vector3();
                //     context.getWorldScale(worldScale);

                //     // the absolute values prevent an error that will cause the cylinder to disappear
                //     // when the worldscale.z goes negative, it also flips the x scale to negative to prevent the cylinder from flipping horizontally
                //     // but this means that worldScale.x + worldScale.y = 0 :(
                //     // so making them abs will solve this
                //     const uniformScale =
                //         (Math.abs(worldScale.x) + Math.abs(worldScale.y)) /
                //         2;
                //     const localScale = context.scale;
                //     context.scale.setX(
                //         (localScale.x / worldScale.x) * uniformScale
                //     );
                //     context.scale.setY(
                //         (localScale.y / worldScale.y) * uniformScale
                //     );
                // };
                const a = Math.abs(vector.getComponent(0));
                const b = Math.abs(vector.getComponent(1));
                const c = vector.getComponent(2);

                if(a !== this.getComponent(0)){
                    super.copy(new THREE.Vector3(a,a,c));
                }else if (b !== this.getComponent(1)){
                    super.copy(new THREE.Vector3(b,b,c));
                }else if (c !== this.getComponent(2)){
                    super.copy(vector);
                }
                break;
            default:
                console.log(this.shape);
                throw Error("shape provided to scale vector is not supported");
                break;
        }
    }
}