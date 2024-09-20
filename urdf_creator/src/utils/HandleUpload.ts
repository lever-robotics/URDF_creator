import * as THREE from "three";
import { GLTF, GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

export async function handleUpload(file: string, type: string) {
    const fileText = await readFile(file);
    const object = await loadFileToObject(fileText, type);
    return object.scene;
}

export function loadFileToObject(fileText: string, type: string): Promise<GLTF> {
    const whichLoader = (type: string) => {
        if (type === "xml") {
            //Unsupported
        } else if (type === "urdf") {
            //Unsupported
            //return XMLtoScene(e.target.result);
        } else if (type === "gltf") {
            return new GLTFLoader();
        } else if (type === "json") {
            // uses parse differently
            // return new THREE.ObjectLoader();
        }
    };
    const loader = whichLoader(type);
    return new Promise((resolve, reject) => {
        loader!.parse(fileText, "", (obj) => {
            resolve(obj);
        });
    });
}

function readFile(file: string): Promise<string> {
    return new Promise((resolve, reject) => {
        const reader = new FileReader();
        reader.onload = () => {
            resolve(reader.result);
        };
        reader.onerror = reject;
        reader.readAsText(file);
    });
}

export async function handleProject(path: string) {
    const loader = new GLTFLoader();
    return new Promise((resolve, reject) => {
        loader.load(path, (obj) => {
            resolve(obj);
        });
    });
}
