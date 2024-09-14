import * as React from "react";
import { useRef, useState } from "react";
import MenuIcon from "./MenuIcon.jsx";
import HelpIcon from "../ApplicationHelp/HelpIcon.jsx";
import "./MenuBar.css";


export default function MenuBar({ stateFunctions, projectTitle }) {

    const {
        openProjectManager,
        openOnboarding,
        openExportDisplayer,
        openImportDisplayer,
        changeProjectTitle,
    } = stateFunctions;

    const [isInvalid, setIsInvalid] = useState(false);

    const handleTitleChange = (event) => {
        let title = event.target.value;
        let altered_title = title.toLowerCase().replace(/[^a-zA-Z0-9_/]/g, "").replace(" ", "_");
        if (/^\d/.test(altered_title)) {
            altered_title = "a" + altered_title.slice(1);
        }

        if (title !== altered_title) {
            setIsInvalid(true);
            setTimeout(() => setIsInvalid(false), 500);
        }

        changeProjectTitle({ target: { value: altered_title } });
    };

    return (
        <div className="menu-bar">
            <MenuIcon
                openExportDisplayer={openExportDisplayer}
                openImportDisplayer={openImportDisplayer}
                openProjectManager={openProjectManager}
            />
            <HelpIcon openOnboarding={openOnboarding} />
            <input
                type="text"
                value={projectTitle}
                className={`project-title-input ${isInvalid ? 'invalid' : ''}`}
                onChange={handleTitleChange}
            />
        </div>
    );
}
