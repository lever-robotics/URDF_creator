import * as React from "react";
import { useState, useRef } from "react";
import MenuIcon from "./MenuIcon";
import HelpIcon from "../ApplicationHelp/HelpIcon";
import "./MenuBar.css";
import { ModalFunctionsType } from "../../App";

export default function MenuBar({ modalFunctions, projectTitle }: {modalFunctions: ModalFunctionsType, projectTitle: string}) {
    const { openProjectManager, openOnboarding, openExportDisplayer, openImportDisplayer, changeProjectTitle } = modalFunctions;

    const [isInvalid, setIsInvalid] = useState(false);

    const handleTitleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
        let title = event.target.value;
        let altered_title = title.toLowerCase().replace(/[^a-zA-Z0-9_/]/g, "").replace(" ", "_");
        if (/^\d/.test(altered_title)) {
            altered_title = "a" + altered_title.slice(1);
        }

        if (title !== altered_title) {
            setIsInvalid(true);
            setTimeout(() => setIsInvalid(false), 500);
        }

        event.target.value = altered_title;
        changeProjectTitle(event);
    };

    return (
        <div className="menu-bar">
            <MenuIcon openExportDisplayer={openExportDisplayer} openImportDisplayer={openImportDisplayer} openProjectManager={openProjectManager} />
            <HelpIcon openOnboarding={openOnboarding} />
            <input type="text" value={projectTitle} 
            className={`project-title-input ${isInvalid ? 'invalid' : ''}`}
            onChange={handleTitleChange}
            />
        </div>
    );
}
