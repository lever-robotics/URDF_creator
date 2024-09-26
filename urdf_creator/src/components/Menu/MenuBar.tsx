import * as React from "react";
import { useState, useRef } from "react";
import MenuIcon from "./MenuIcon";
import HelpIcon from "../ApplicationHelp/HelpIcon";
import "./MenuBar.css";
import { ModalFunctionsType } from "../../App";

export default function MenuBar({ modalFunctions, projectTitle }: {modalFunctions: ModalFunctionsType, projectTitle: string}) {
    const { openProjectManager, openOnboarding, openExportDisplayer, openImportDisplayer, changeProjectTitle } = modalFunctions;

    return (
        <div className="menu-bar">
            <MenuIcon openExportDisplayer={openExportDisplayer} openImportDisplayer={openImportDisplayer} openProjectManager={openProjectManager} />
            <HelpIcon openOnboarding={openOnboarding} />
            <input type="text" value={projectTitle} className="project-title-input" onChange={changeProjectTitle} />
        </div>
    );
}
