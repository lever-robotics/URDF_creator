import * as React from "react";
import * as THREE from "three";
import { useState, useRef } from "react";
import Button from "@mui/material/Button";
import Menu from "@mui/material/Menu";
import MenuItem from "@mui/material/MenuItem";
import { styled } from "@mui/material/styles";
import PopupState, { bindTrigger, bindMenu } from "material-ui-popup-state";
import { handleDownload } from "../../utils/HandleDownload";
import { handleUpload } from "../../utils/HandleUpload";
import "./MenuModal.css";

export default function MenuModal({
  openProjectManager,
  changeProjectTitle,
  projectTitle,
  getScene,
  loadScene,
}) {
  const inputFile = useRef(null);
  const [selectedFile, setSelectedFile] = useState(null);

  const StyledMenuItem = styled((props) => (
    <MenuItem {...props} disableRipple></MenuItem>
  ))(({ theme }) => ({
    minWidth: 0,
    fontSize: "3em",
    fontWeight: 200,
    fontFamily: "inherit",
    transition: "font-size 0.4s",
    "&:hover, &.Mui-focusVisible": {
      fontSize: "4em",
    },
  }));

  const StyledMenu = styled((props) => <Menu {...props} />)(({ theme }) => ({
    "& .MuiPaper-root": {
      background: "transparent",
      color: "white",
      boxShadow: "none",
      height: "100%",
      width: "30%",
    },
    "& .MuiList-root": {
      textOverflow: "ellipsis",
      height: "90%",
      display: "flex",
      paddingLeft: "15%",
      justifyContent: "space-evenly",
      flexDirection: "column",
    },
    background: "rgba(0, 0, 0, 0.6)",
  }));

  const downloadURDF = (scene, projectTitle) =>
    handleDownload(scene, "urdf", projectTitle);

  const downloadJSON = (scene, projectTitle) => {
    handleDownload(scene, "gltf", projectTitle);
  };

  function JSONtoScene(jsonstring) {
    const json = JSON.parse(jsonstring);
    console.log(json);
    const loader = new THREE.ObjectLoader();
    loader.parse(json, (obj) => {
      loadScene(obj);
    });
  }

  /* Annoying File Upload Logic
  1. Clicking Upload File activates onFileUpload() which 'clicks' the input element
  2. The input element has an onChange listener that uploads the file using the handleFileChange() function which calls handleUpload() */
  const onFileUpload = () => inputFile.current.click();
  const handleFileChange = (e) => {
    handleUpload(e.target.files[0]).then((scene) => {
      loadScene(scene);
    });
  };

  return (
    <PopupState variant="popover" popupId="demo-popup-menu">
      {(popupState) => (
        <React.Fragment>
          <div className="menu">
            <Button
              variant="contained"
              {...bindTrigger(popupState)}
              class="material-symbols-outlined"
            >
              menu
            </Button>
            <input
              type="text"
              value={projectTitle}
              id="projectTitleInput"
              onChange={changeProjectTitle}
            />
          </div>
          <StyledMenu {...bindMenu(popupState)}>
            <StyledMenuItem
              onClick={() => {
                openProjectManager();
                popupState.close();
              }}
            >
              Project Manager
            </StyledMenuItem>
            <StyledMenuItem
              onClick={() => {
                downloadURDF(getScene(), projectTitle);
                popupState.close();
              }}
            >
              Export URDF
            </StyledMenuItem>
            <StyledMenuItem
              onClick={() => {
                downloadJSON(getScene(), projectTitle);
                popupState.close();
              }}
            >
              Export JSON
            </StyledMenuItem>
            <StyledMenuItem
              onClick={() => {
                onFileUpload();
                popupState.close();
              }}
            >
              Upload File
            </StyledMenuItem>
          </StyledMenu>
          <input
            type="file"
            ref={inputFile}
            style={{ display: "none" }}
            onChange={handleFileChange}
          />
        </React.Fragment>
      )}
    </PopupState>
  );
}
