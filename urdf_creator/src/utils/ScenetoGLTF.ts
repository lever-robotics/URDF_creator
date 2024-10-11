import type * as THREE from "three";
import { GLTFExporter } from "three/examples/jsm/exporters/GLTFExporter";

export default async function ScenetoGLTF(scene: THREE.Object3D) {
    const exporter = new GLTFExporter();
    return exporter.parseAsync(scene);
}
