import type * as THREE from "three";
import Frame from "../Models/Frame";

export default function findRootFrame(scene: THREE.Object3D) {
    if (scene) {
        if (scene.children) {
            const children = scene.children.filter((child) => {
                return child instanceof Frame;
            });
            if (children.length > 0) {
                return children[0];
            }
        }
    }
}
