import * as React from "react";
import * as THREE from "three";
import { useState, useRef } from "react";
import MenuIcon from "./MenuIcon.jsx";
import PopupState, { bindTrigger, bindMenu } from "material-ui-popup-state";
import HelpIcon from "../ApplicationHelp/HelpIcon.jsx";
import { handleDownload } from "../../utils/HandleDownload.js";
import { handleUpload } from "../../utils/HandleUpload.js";
import { openDB } from "idb";
import { StyledMenu, StyledButton, StyledMenuItem } from "./StyledItems.jsx";
import "./MenuModal.css";
import ReactGA from "react-ga4";
import urdfObjectManager from "../../Models/urdfObjectManager.js";
import MenuModal from "../../FunctionalComponents/MenuModal.jsx";

export default function MenuBar({ stateFunctions, projectTitle }) {
    const inputFile = useRef(null);
    const inputSTLFile = useRef(null);

    const [isModalOpen, setIsModalOpen] = useState(false);
    const [modalPosition, setModalPosition] = useState({ top: 0, left: 0 });
    const buttonRef = useRef(null);

    const menuItems = [
        { label: "Profile", action: () => alert("Profile clicked!") },
        { label: "Settings", action: () => alert("Settings clicked!") },
        { label: "Logout", action: () => alert("Logout clicked!") },
    ];

    const {
        openProjectManager,
        openOnboarding,
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

    const openMainMenu = () => {
        console.log("I am a button");
    };

    const openModal = () => {
        const buttonRect = buttonRef.current.getBoundingClientRect();
        setModalPosition({
            top: buttonRect.bottom + window.scrollY, // Position below the button
            left: buttonRect.left + window.scrollX, // Align left with the button
        });
        setIsModalOpen(true);
    };

    const closeModal = () => {
        setIsModalOpen(false);
    };

    // Consider creating a menuItemFactory to map menuItems out. Then all the menu items can be defined above and implemented below. Could be a more extendable and easier to read approach

    // Unrelated to this class but consider creating a urdfBuilder class. This might make it more extendable in the future because we may add urdfObjects in lots of different ways and configurations. Instead of storing all that functionality in the urdfObject itself, the construction knowledge would be kept in the builder. I think this could be useful for "addUrdfObject" "loadScene" and for eventually handling different formats

    return (
        <div className="menu">
            <MenuIcon openMainMenu={openModal} ref={buttonRef} />
            <HelpIcon openOnboarding={openOnboarding} />
            <MenuModal
                isOpen={isModalOpen}
                onClose={closeModal}
                menuItems={menuItems}
                position={modalPosition}
            />
        </div>
        // <PopupState variant="popover" popupId="demo-popup-menu">
        //     {(popupState) => (
        //         <React.Fragment>
        //             <div className="menu">
        //                 <StyledButton
        //                     variant="contained"
        //                     {...bindTrigger(popupState)}
        //                     className="material-symbols-outlined">
        //                     <MenuIcon />
        //                 </StyledButton>
        //                 <HelpIcon openOnboarding={openOnboarding} />
        //                 <input
        //                     type="text"
        //                     value={projectTitle}
        //                     id="projectTitleInput"
        //                     onChange={changeProjectTitle}
        //                 />
        //             </div>
        //             <StyledMenu {...bindMenu(popupState)}>
        //                 <StyledMenuItem
        //                     onClick={() => {
        //                         openProjectManager();
        //                         popupState.close();
        //                     }}>
        //                     Project Manager
        //                 </StyledMenuItem>
        //                 <StyledMenuItem
        //                     onClick={() => {
        //                         ReactGA.event({
        //                             category: "Button", // Typically the object that was interacted with
        //                             action: "Click", // The type of interaction
        //                             label: "Downloads of URDF package", // Optional, can be used to identify specific buttons
        //                         });
        //                         handleDownload(
        //                             getScene(),
        //                             "urdf",
        //                             projectTitle
        //                         );
        //                         popupState.close();
        //                     }}>
        //                     Export URDF
        //                 </StyledMenuItem>
        //                 <StyledMenuItem
        //                     onClick={() => {
        //                         handleGLTFExport();
        //                         popupState.close();
        //                     }}>
        //                     Export GLTF
        //                 </StyledMenuItem>
        //                 <StyledMenuItem
        //                     onClick={() => {
        //                         onFileUpload();
        //                         popupState.close();
        //                     }}>
        //                     Upload File
        //                 </StyledMenuItem>
        //                 {/* Disabling STL upload for now */}
        //                 {/* <StyledMenuItem
        //                     onClick={() => {
        //                         onSTLFileUpload();
        //                         popupState.close();
        //                     }}>
        //                     Upload STL
        //                 </StyledMenuItem> */}
        //             </StyledMenu>
        //             <input
        //                 type="file"
        //                 ref={inputFile}
        //                 style={{ display: "none" }}
        //                 onChange={handleFileChange}
        //             />
        //             <input
        //                 type="file"
        //                 ref={inputSTLFile}
        //                 style={{ display: "none" }}
        //                 onChange={handleSTLFileChange}
        //                 accept=".stl"
        //             />
        //         </React.Fragment>
        //     )}
        // </PopupState>
    );
}
