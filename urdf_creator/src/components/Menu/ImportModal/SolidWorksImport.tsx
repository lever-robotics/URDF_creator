import React from 'react';
import './Import.css';
import { useRef } from 'react';

const SolidWorksImport = ({ onClose }: {onClose: () => void}) => {


    const onClick = () => {
        //send user to websiite
        window.location.href = 'https://www.roboeverything.com/';

        onClose();
    }

    return (
        <div className='des-container'>
            <div className="header">
                <div className="import-title">
                    <h3>Import from Solidworks</h3>
                </div>
                <div className="import-box">
                    <button className='import-button' onClick={onClick}>Import from Solidworks</button>
                </div>
            </div>
            <div className="description-container">
                <h3>Description: </h3>
                <p>Robot models can be easily imported from SolidWorks using the Solidworks to URDF plugin tool. Your SolidWorks kinematics and dynamic properties can be ported over and additional frames, sensor configurations can be prepared here for simulation.</p>
            </div>
            <div className="image-description-container">
                <div className="image-container">
                    <img src={'/statics/STL_Description.png'} alt="STL Mesh" className="stlgraphic" />
                </div>
            </div>
            <div className="description-container">
                <p>For converting 3D models from CAD software to STL or DAE formats, refer to the <a href="https://roboeverything.com">documentation</a>.</p>
            </div>
        </div>
    );
}

export default SolidWorksImport;