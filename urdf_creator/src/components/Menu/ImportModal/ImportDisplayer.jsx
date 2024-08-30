import React, { useState, useRef } from 'react';
import urdfObjectManager from '../../../Models/urdfObjectManager';
import { handleDownload } from '../../../utils/HandleDownload';
import { handleUpload } from '../../../utils/HandleUpload';
import GltfFilesGrid from '../Import/ImportSensor';
import { openDB } from "idb";
import ReactGA from "react-ga4";

import './importDisplayer.css';

const ImportDisplayer = ({ onClose, loadScene }) => {
    const inputFile = useRef(null);
    const inputSTLFile = useRef(null);

    const [content, setContent] = useState("");

    /* Annoying File Upload Logic
      1. Clicking Upload File activates onFileUpload() which 'clicks' the input element
      2. The input element has an onChange listener that uploads the file using the handleFileChange() function which calls handleUpload() 
    */
      const onFileUpload = () => inputFile.current.click();
      const handleFileChange = async (e) => {
          const file = e.target.files[0];
          const type = file.name.split(".").pop();
          const group = await handleUpload(file, type);
          const base_link = group.children[0];
          loadScene(base_link);
      };
  
      const onSTLFileUpload = () => inputSTLFile.current.click();
      const handleSTLFileChange = async (event) => {
          const selectedFile = event.target.files[0];
          if (selectedFile) {
              const db = await openDB("stlFilesDB", 1, {
                  upgrade(db) {
                      if (!db.objectStoreNames.contains("files")) {
                          db.createObjectStore("files", { keyPath: "name" });
                      }
                  },
              });
              await db.put("files", {
                  name: selectedFile.name,
                  file: selectedFile,
              });
  
              event.target.value = null; // Clear the input value after upload
          }
      };

    const importOptions = [
        { label: "STL", action: () => {}, content: "A STL file"},
        { label: "Robot Sensor", action: () => {}, content: {GltfFilesGrid}},
        { label: "GLTF", action: () => {onFileUpload(); onClose();}, content: "Upload a project"}
    ]



    return (
        <>
            <h2 className="title">Import Options</h2>
            <div className="import-displayer">
                <ul className="import-list">
                    {importOptions.map((item, index) => (
                        <ImportOption index={index} item={item} setContent={setContent}/>
                    ))}
                </ul>
                <div className="content">
                    {content}
                </div>
            </div>
            <input
                type="file"
                ref={inputFile}
                style={{ display: "none" }}
                onChange={handleFileChange}
            />
            <input
                type="file"
                ref={inputSTLFile}
                style={{ display: "none" }}
                onChange={handleSTLFileChange}
                accept=".stl"
            />
        </>
        
    );
};

const ImportOption = ({ item, setContent }) => {

    return (
        <li className="import-option" onClick={item.action} onMouseEnter={() => setContent(item.content)} onMouseLeave={() => setContent("")}>
            <span className="import-option-span">
                {item.label}
            </span>
        </li>
    )
}

export default ImportDisplayer;