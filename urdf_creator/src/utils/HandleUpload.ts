import * as THREE from "three";
import { type GLTF, GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";

export async function handleUpload(file: Blob, type: string) {
    const fileText = await readFile(file);
    const object = await loadFileToObject(fileText, type);
    return object.scene;
}

export function loadFileToObject(
    fileText: string,
    type: string,
): Promise<GLTF> {
    // const whichLoader = (type: string) => {
    //     if (type === "xml") {
    //         //Unsupported
    //     } else if (type === "urdf") {
    //         //Unsupported
    //         //return XMLtoScene(e.target.result);
    //     } else {
    //         return new GLTFLoader();
    //     }
    //     // else if (type === "json") {
    //     //     // uses parse differently
    //     //     // return new THREE.ObjectLoader();
    //     // }
    // };
    // const loader = whichLoader(type);
    const loader = new GLTFLoader();
    return new Promise((resolve, reject) => {
        loader.parse(fileText, "", (obj) => {
            resolve(obj);
        });
    });
}

function readFile(file: Blob): Promise<string> {
    return new Promise((resolve, reject) => {
        const reader = new FileReader();
        reader.onload = () => {
            resolve(reader.result as string);
        };
        reader.onerror = reject;
        reader.readAsText(file);
    });
}

export async function handleProject(path: string) {
    const loader = new GLTFLoader();
    return new Promise<GLTF>((resolve, reject) => {
        loader.load(path, (obj) => {
            resolve(obj);
        });
    });
}
