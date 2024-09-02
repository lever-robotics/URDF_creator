import React, { useState, useRef } from 'react';
import urdfObjectManager from '../../../Models/urdfObjectManager';
import STLImport from './STLImport';
import GLTFImport from './GLTFImport';
import GltfFilesGrid from './ImportSensor';
import ReactGA from "react-ga4";

import "./importDisplayer.css";

const ImportDisplayer = ({ handleSensorClick, onImportClose, loadScene }) => {
    const [content, setContent] = useState(<GLTFImport onClose={onImportClose} loadScene={loadScene} />);
    const [selectedIndex, setSelectedIndex] = useState(1);

    const importOptions = [
        { label: "STL", content: <STLImport onClose={onImportClose} /> },
        { label: "GLTF", content: <GLTFImport onClose={onImportClose} loadScene={loadScene} />},
        { label: "Robot Sensor", content: <GltfFilesGrid handleSensorClick={handleSensorClick}/> },
        // { label: "Sensors", content: },
        // { label: "Link", content: },
        // { label: "URDF", content: },
    ];

    return (
        <>
            <h2 className="title">Import Options</h2>
            <div className="import-displayer">
                <div className='import-menu-modal'>                
                    <ul className="menu-list">
                    {importOptions.map((item, index) => (
                        <ImportOption 
                        key={index}
                        index={index} 
                        item={item} 
                        setContent={setContent} 
                        selectedIndex={selectedIndex}
                        setSelectedIndex={setSelectedIndex}
                    />
                    ))}
                    </ul>
                </div>
                {content}
            </div>
        </>
    );
};

const ImportOption = ({ item, index, setContent, selectedIndex, setSelectedIndex }) => {

    const handleClick = () => {
        setContent(item.content);
        setSelectedIndex(index);
    };

    return (
        <li 
            className={`menu-item ${selectedIndex === index ? 'menu-selected' : ''}`} 
            onClick={handleClick}
        >
            {item.label}
        </li>
    );
};

export default ImportDisplayer;
