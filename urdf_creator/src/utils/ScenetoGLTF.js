import { GLTFExporter } from "three/addons/exporters/GLTFExporter.js";


export default async function ScenetoGLTF(scene){
    const exporter = new GLTFExporter();
    return exporter.parseAsync(scene);
}