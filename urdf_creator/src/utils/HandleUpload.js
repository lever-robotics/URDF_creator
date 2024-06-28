import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader';

export async function handleUpload(file, type) {
    const fileText = await readFile(file);
    const object = await loadFileToObject(fileText, type);
    return object.scene;
}

function loadFileToObject(fileText, type) {
    const whichLoader = (type) => {
        if (type === 'xml') {
            //Unsupported
        } else if (type === 'urdf') {
            //Unsupported
            //return XMLtoScene(e.target.result);
        } else if (type === 'gltf') {
            return new GLTFLoader();
        } else if (type === 'json') {
            // uses parse differently
            // return new THREE.ObjectLoader();
        }
    };
    const loader = whichLoader(type);
    return new Promise((resolve, reject) => {
        loader.parse(fileText, '', (obj) => {
            resolve(obj);
        });
    });
}

function readFile(file) {
    return new Promise((resolve, reject) => {
        const reader = new FileReader();
        reader.onload = () => {
            resolve(reader.result);
        };
        reader.onerror = reject;
        reader.readAsText(file);
    }) 
  } 

export async function handleProject(path){
    const loader = new GLTFLoader();
    return new Promise((resolve, reject) => {
        loader.load(path, (obj) => {
            resolve(obj);
        });
    });
}