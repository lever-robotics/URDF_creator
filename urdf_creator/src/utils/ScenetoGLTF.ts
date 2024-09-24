import { GLTFExporter } from "three/examples/jsm/exporters/GLTFExporter.js";
import * as THREE from "three";


export default async function ScenetoGLTF(scene: THREE.Object3D){
    const exporter = new GLTFExporter();
    return exporter.parseAsync(scene);
}