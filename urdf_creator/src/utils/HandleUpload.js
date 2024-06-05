import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader';

export async function handleUpload(file) {
    const type = file.name.split('.').pop();
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
    });
}

// function JSONtoScene(jsonstring) {
//     const json = JSON.parse(jsonstring);
//     console.log(json);
//     const loader = new THREE.ObjectLoader();
//     loader.parse(json, (obj) => {
//         loadScene(obj);
//     });
// }

// const loadSTL = (file) => {
//     const reader = new FileReader();
//     reader.onload = (event) => {
//         const geometry = new STLLoader().parse(event.target.result);
//         const material = new THREE.MeshPhongMaterial({ color: Math.random() * 0xffffff });
//         const stlMesh = new THREE.Mesh(geometry, material);

//         // Compute the bounding box of the STL geometry
//         const bbox = new THREE.Box3().setFromObject(stlMesh);
//         const size = new THREE.Vector3();
//         bbox.getSize(size);

//         // Create a cube that bounds the STL geometry
//         const cubeGeometry = new THREE.BoxGeometry(size.x, size.y, size.z);
//         const cubeMaterial = new THREE.MeshPhongMaterial({ color: 0x888888, wireframe: true });
//         const cubeMesh = new THREE.Mesh(cubeGeometry, cubeMaterial);

//         // Position the STL geometry inside the cube
//         stlMesh.position.set(-size.x / 2, -size.y / 2, -size.z / 2);
//         cubeMesh.add(stlMesh);

//         // Set userData for cubeMesh
//         cubeMesh.userData = {
//             customInertia: false,
//             Ixx: 0,
//             Ixy: 0,
//             Ixz: 0,
//             Iyy: 0,
//             Iyz: 0,
//             Izz: 0,
//             isBaseLink: false,
//             isSensor: false,
//             mass: 1,
//             name: 'stl_cube',
//             scaler: new THREE.Object3D(),
//             selectable: true,
//             sensorType: "",
//             shape: 'cube',
//             // IMU Parameters
//             gaussianNoise: 0.01,
//             xyzOffsets: '0 0 0',
//             rpyOffsets: '0 0 0',
//             alwaysOn: true,
//             updateRate: 100,
//             mean: 0,
//             stddev: 0,
//             // Camera Parameters
//             cameraName: 'camera',
//             imageTopicName: '/camera/image_raw',
//             cameraInfoTopicName: '/camera/camera_info',
//             horizontal_fov: 1.3962634,
//             width: 800,
//             height: 600,
//             format: 'R8G8B8',
//             near: 0.1,
//             far: 100
//             // Add additional sensor parameters here
//         };

//         addObjectToScene(cubeMesh);
//     };
//     reader.readAsArrayBuffer(file);
// };

// const loadDAE = (file) => {
//     const reader = new FileReader();
//     reader.onload = (event) => {
//         const loader = new ColladaLoader();
//         loader.parse(event.target.result, (collada) => {
//             const model = collada.scene;
//             addObjectToScene(model);
//         });
//     };
//     reader.readAsText(file);
// };
