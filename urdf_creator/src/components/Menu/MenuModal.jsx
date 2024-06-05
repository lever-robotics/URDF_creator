import * as React from 'react';
import * as THREE from 'three';
import { useState, useRef } from 'react';
import Button from '@mui/material/Button';
import Menu from '@mui/material/Menu';
import MenuIcon from '@mui/icons-material/Menu';
import MenuItem from '@mui/material/MenuItem';
import { styled } from '@mui/material/styles';
import PopupState, { bindTrigger, bindMenu } from 'material-ui-popup-state';
import { handleDownload } from '../../utils/HandleDownload';
import { handleUpload } from '../../utils/HandleUpload';
import './MenuModal.css';

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
        fontSize: '3em',
        fontWeight: 200,
        fontFamily: 'inherit',
        transition: 'font-size 0.4s',
        '&:hover, &.Mui-focusVisible': {
            fontSize: '4em',
        },
    }));

    const StyledMenu = styled((props) => <Menu {...props} />)(({ theme }) => ({
        '& .MuiPaper-root': {
            background: 'transparent',
            color: 'white',
            boxShadow: 'none',
            height: '100%',
            width: '30%',
        },
        '& .MuiList-root': {
            textOverflow: 'ellipsis',
            height: '90%',
            display: 'flex',
            paddingLeft: '15%',
            justifyContent: 'space-evenly',
            flexDirection: 'column',
        },
        background: 'rgba(0, 0, 0, 0.6)',
    }));

    const StyledButton = styled((props) => <Button {...props} />)(({ theme }) => ({
        borderRadius: '8px',
        border: '1px solid transparent',
        padding: '0.6em 1.2em',
        fontSize: '1em',
        fontWeight: 500,
        fontFamily: 'inherit',
        backgroundColor: '#1d2a31',
        cursor: 'pointer',
        transition: 'border-color 0.25s',
        '&:hover, .Mui-focusVisible, .MuiButton-colorSuccess': {
          borderColor: '#646cff',
          backgroundColor: '#1d2a31',
        }
    }));

    /* Annoying File Upload Logic
      1. Clicking Upload File activates onFileUpload() which 'clicks' the input element
      2. The input element has an onChange listener that uploads the file using the handleFileChange() function which calls handleUpload() 
    */
    const onFileUpload = () => inputFile.current.click();
    const handleFileChange = async (e) => {
        const scene = await handleUpload(e.target.files[0]);
        loadScene(scene);
    };

    return (
        <PopupState variant="popover" popupId="demo-popup-menu">
            {(popupState) => (
                <React.Fragment>
                    <div className="menu">
                        <StyledButton
                            variant="contained"
                            {...bindTrigger(popupState)}
                            className="material-symbols-outlined">
                            <MenuIcon/>
                        </StyledButton>
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
                            }}>
                            Project Manager
                        </StyledMenuItem>
                        <StyledMenuItem
                            onClick={() => {
                                handleDownload(
                                    getScene(),
                                    'urdf',
                                    projectTitle
                                );
                                popupState.close();
                            }}>
                            Export URDF
                        </StyledMenuItem>
                        <StyledMenuItem
                            onClick={() => {
                                handleDownload(
                                    getScene(),
                                    'gltf',
                                    projectTitle
                                );
                                popupState.close();
                            }}>
                            Export GLTF
                        </StyledMenuItem>
                        <StyledMenuItem
                            onClick={() => {
                                onFileUpload();
                                popupState.close();
                            }}>
                            Upload File
                        </StyledMenuItem>
                    </StyledMenu>
                    <input
                        type="file"
                        ref={inputFile}
                        style={{ display: 'none' }}
                        onChange={handleFileChange}
                    />
                </React.Fragment>
            )}
        </PopupState>
    );
}
