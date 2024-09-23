import { GLTFExporter } from "three/addons/exporters/GLTFExporter.js";
import * as THREE from "three";


export default async function ScenetoGLTF(scene: THREE.Scene){
    const exporter = new GLTFExporter();
    return exporter.parseAsync(scene);
}