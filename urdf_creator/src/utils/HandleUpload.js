// import * as THREE from "three";
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';


export function handleUpload(file){
    const type = file.name.split('.').pop();
    console.log(type);
    return new Promise((resolve, reject) => {
        const reader = new FileReader();
        reader.onload = (e) => {
            if(type === 'xml'){
                //Unsupported
            }else if(type === 'urdf'){
                //Unsupported
                //return XMLtoScene(e.target.result);
            }else if (type === 'gltf'){
                const loader = new GLTFLoader();
                loader.parse(e.target.result,'', (gltf) => {
                    resolve(gltf.scene);
                });
                // console.log(e.target.result);
                // const scene = JSONtoScene(e.target.result);
                // console.log(scene);
                // loadScene(scene);
            }
        }
        reader.onerror = reject;
        reader.readAsText(file);
    })
  } 