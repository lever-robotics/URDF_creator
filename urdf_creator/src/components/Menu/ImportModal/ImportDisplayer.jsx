import React, { useState, useRef } from 'react';
import urdfObjectManager from '../../../Models/urdfObjectManager';
import { handleDownload } from '../../../utils/HandleDownload';
import STLImport from './STLImport';
import { handleUpload } from '../../../utils/HandleUpload';
import { openDB } from "idb";
import ReactGA from "react-ga4";

import './importDisplayer.css';
import '../../../FunctionalComponents/MenuModal.css';

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
        { label: "STL", action: () => {}, content: <STLImport /> },
        // { label: "Robot Package", action: () => {}, content: "Download the whole Robot Package necessary for ROS2"},
        { label: "GLTF", action: () => {}, content: "Upload a project"}
    ];

    return (
        <>
            <h2 className="title">Import Options</h2>
            <div className="import-displayer">
                <ul className="menu-list">
                    {importOptions.map((item, index) => (
                        <ImportOption index={index} item={item} setContent={setContent}/>
                    ))}
                </ul>
                {content}
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
        <li className="menu-item" onClick={() => setContent(item.content)}>
            {item.label}
        </li>
    )
}

export default ImportDisplayer;