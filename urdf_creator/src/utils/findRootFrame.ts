import Frame from "../Models/Frame";
import * as THREE from "three";

export default function findRootFrame(scene: THREE.Scene) {
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
