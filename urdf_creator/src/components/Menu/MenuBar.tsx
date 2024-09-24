import * as React from "react";
import { useState, useRef } from "react";
import MenuIcon from "./MenuIcon";
import HelpIcon from "../ApplicationHelp/HelpIcon";
import "./MenuBar.css";
import { StateFunctionsType } from "../SceneState";

export default function MenuBar({ stateFunctions, projectTitle }: {stateFunctions: StateFunctionsType, projectTitle: string}) {
    const { openProjectManager, openOnboarding, openExportDisplayer, openImportDisplayer, changeProjectTitle } = stateFunctions;

    return (
        <div className="menu-bar">
            <MenuIcon openExportDisplayer={openExportDisplayer} openImportDisplayer={openImportDisplayer} openProjectManager={openProjectManager} />
            <HelpIcon openOnboarding={openOnboarding} />
            <input type="text" value={projectTitle} className="project-title-input" onChange={changeProjectTitle} />
        </div>
    );
}
