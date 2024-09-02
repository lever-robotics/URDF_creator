import React from 'react';
import './GLTFImport.css';
import { useRef } from 'react';
import { handleUpload } from '../../../utils/HandleUpload';


const GLTFImport = ({ onClose, loadScene }) => {
    const inputGTLFFile = useRef(null);

    /* Annoying File Upload Logic
      1. Clicking Upload File activates onFileUpload() which 'clicks' the input element
      2. The input element has an onChange listener that uploads the file using the handleFileChange() function which calls handleUpload() 
    */
      const onFileUpload = () => inputGTLFFile.current.click();
      const handleFileChange = async (e) => {
          const file = e.target.files[0];
          const type = file.name.split(".").pop();
          const group = await handleUpload(file, type);
          const base_link = group.children[0];
          loadScene(base_link);
      };

    const onClick = () => {
        onFileUpload();
        onClose();
    };

    return (
        <div className='gltf-container'>
            <div className="header">
                <div className="gltf-import-title">
                    <h3>Import a GLTF file</h3>
                </div>
                <div className="import-box">
                    <button className='gltf-import-button' onClick={onClick}>Import GLTF</button>
                </div>
            </div>
            <div className="description-container">
                <h3>Description: </h3>
                <p>A GLTF file is a local file that you would have exported from this URDF creator to save a local copy of your project. It takes all the information from your defined robot and condenses it into a small file that can be imported here.</p>
            </div>
            <div className="image-description-container">
                <div className="image-container">
                    <img src={process.env.PUBLIC_URL + '/statics/gltf_share.png'} alt="gltf Share" className="gltfgraphic" />
                </div>
                <div className="image-description">Share your robot description in this software with developer teams to modify the same robot description</div>
            </div>
            <div className="description-container">
                <p>For understanding the capabilities of GLTF files in versioning your defined robot, refer to the <a href="https://roboeverything.com">documentation</a>.</p>
            </div>
            <input
                type="file"
                ref={inputGTLFFile}
                style={{ display: "none" }}
                onChange={handleFileChange}
                accept='.gltf'
            />
        </div>
    );
}

export default GLTFImport;