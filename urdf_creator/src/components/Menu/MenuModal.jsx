import * as React from 'react';
import { useState } from 'react';
import Button from '@mui/material/Button';
import Menu from '@mui/material/Menu';
import MenuItem from '@mui/material/MenuItem';
import PopupState, { bindTrigger, bindMenu } from 'material-ui-popup-state';
import './MenuModal.css'

export default function MenuModal() {
  const [projectTitle, setProjectTitle] = useState('untitled');

  return (
    <PopupState variant="popover" popupId="demo-popup-menu">
      {(popupState) => (
        <React.Fragment>
          <div className='menu'>
            <Button variant="contained" {...bindTrigger(popupState)} class="material-symbols-outlined">
              menu
            </Button>
            <div Id='projectTitle'>{projectTitle}</div>
          </div>
          <Menu {...bindMenu(popupState)}>
            <MenuItem className='MenuItem' onClick={popupState.close}>Profile</MenuItem>
            <MenuItem className='MenuItem' onClick={popupState.close}>My account</MenuItem>
            <MenuItem className='MenuItem' onClick={popupState.close}>Logout</MenuItem>
          </Menu>
        </React.Fragment>
      )}
    </PopupState>
  );
}

           
           
