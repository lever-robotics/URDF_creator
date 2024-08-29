import * as React from "react";
import { useState, useRef } from "react";
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
        getBaseLink,
        getScene,
        loadScene,
    } = stateFunctions;



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
                className="project-title-input"
                onChange={changeProjectTitle}
            />
        </div>
    );
}
