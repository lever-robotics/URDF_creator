import React, { useRef } from "react";
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader';
import * as THREE from 'three';

function InsertTool({ addObject, addObjectToScene }) {
    const fileInputRefSTL = useRef(null);
    const fileInputRefDAE = useRef(null);

    const handleFileButtonClick = (ref) => {
        if (ref.current) {
            ref.current.click();
        }
    };

    const loadSTL = (file) => {
        const reader = new FileReader();
        reader.onload = (event) => {
            const geometry = new STLLoader().parse(event.target.result);
            const material = new THREE.MeshPhongMaterial({ color: Math.random() * 0xffffff });
            const mesh = new THREE.Mesh(geometry, material);
            addObjectToScene(mesh);
        };
        reader.readAsArrayBuffer(file);
    };

    const loadDAE = (file) => {
        const reader = new FileReader();
        reader.onload = (event) => {
            const loader = new ColladaLoader();
            loader.parse(event.target.result, (collada) => {
                const model = collada.scene;
                addObjectToScene(model);
            });
        };
        reader.readAsText(file);
    };

    const handleFileUpload = (event, loaderFunction) => {
        const file = event.target.files[0];
        if (file) {
            loaderFunction(file);
        }
    };

    return (
        <div style={{ marginTop: "10px" }} className="column-box">
            Add Objects
            <button onClick={() => addObject("cube")}>Add Cube</button>
            <button onClick={() => addObject("sphere")}>Add Sphere</button>
            <button onClick={() => addObject("cylinder")}>Add Cylinder</button>
            <button onClick={() => handleFileButtonClick(fileInputRefSTL)}>Upload STL</button>
            <button onClick={() => handleFileButtonClick(fileInputRefDAE)}>Upload DAE</button>
            <input
                type="file"
                accept=".stl"
                ref={fileInputRefSTL}
                onChange={(event) => handleFileUpload(event, loadSTL)}
                style={{ display: 'none' }}
            />
            <input
                type="file"
                accept=".dae"
                ref={fileInputRefDAE}
                onChange={(event) => handleFileUpload(event, loadDAE)}
                style={{ display: 'none' }}
            />
        </div>
    );
}

export default InsertTool;
