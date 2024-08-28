import * as React from "react";
import { useState, useRef } from "react";
import MenuIcon from "./MenuIcon.jsx";
import HelpIcon from "../ApplicationHelp/HelpIcon.jsx";
import { handleDownload } from "../../utils/HandleDownload.js";
import { handleUpload } from "../../utils/HandleUpload.js";
import { openDB } from "idb";
import "./MenuBar.css";
import ReactGA from "react-ga4";
import urdfObjectManager from "../../Models/urdfObjectManager.js";

export default function MenuBar({ stateFunctions, projectTitle }) {
    const inputFile = useRef(null);
    const inputSTLFile = useRef(null);

    const {
        openProjectManager,
        openOnboarding,
        openExportDisplayer,
        changeProjectTitle,
        getBaseLink,
        getScene,
        loadScene,
    } = stateFunctions;

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

    const handleGLTFExport = () => {
        const manager = new urdfObjectManager();
        const compressedScene = manager.compressScene(getBaseLink());
        handleDownload(compressedScene, "gltf", projectTitle);
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

    return (
        <div className="menu-bar">
            <MenuIcon
                openExportDisplayer={openExportDisplayer}
                openProjectManager={openProjectManager}
            />
            <HelpIcon openOnboarding={openOnboarding} />
            <input
                type="text"
                value={projectTitle}
                className="project-title-input"
                onChange={changeProjectTitle}
            />
        </div>
    );
}
