$old = @"
            // ???final_ok??????asso_flag???????
            if(final_ok){
                track_asso_data[best_idx*3+0].asso_flag = 0;
                track_asso_data[best_idx*3+1].asso_flag = 0;
                track_asso_data[best_idx*3+2].asso_flag = 0;

                // ????????
                candidates_left--;
            }
        } // end if(final_ok) for outer if(final_ok) block

        // ?????????????????????? temp_track
        if(candidates_left <= 0 || (*reliable_track_num) >= MAX_RELIABLE_TRACKS){
            break;  // ??????
        }
        } // end while(candidates_left > 0)
        printf("    [DBG new_rel] after_while k_begin=%d num_cand=%d\n", k, num_candidates);
        fflush(stdout);
"@
$new = @"
            // ???final_ok??????asso_flag???????
            if(final_ok){
                track_asso_data[best_idx*3+0].asso_flag = 0;
                track_asso_data[best_idx*3+1].asso_flag = 0;
                track_asso_data[best_idx*3+2].asso_flag = 0;

                // ????????
                candidates_left--;
            }
        } /* end if(best_idx >= 0)  (outer block around final_ok / Kalman / init_new_track) */

        /* Safety break: no unprocessed candidates left OR reliable pool full */
        if(candidates_left <= 0 || (*reliable_track_num) >= MAX_RELIABLE_TRACKS){
            break;
        }
        /* ===== [BUG21 FIX - REAL while-close] ===================================
         * while-loop is ONLY for upgrading 3-point candidates into reliable tracks.
         * All writeback into temp_track MUST happen AFTER the while-loop, otherwise
         * `if(!found_any) break;` on count==1 frames jumps over writeback entirely
         * (was previously enclosed inside while body -> temp_before=0 every frame).
         * ====================================================================== */
    } /* END while(candidates_left > 0 && reliable < MAX_RELIABLE_TRACKS) */

    printf("    [DBG new_rel] after_while k_begin=%d num_cand=%d temp_before=%d reliable=%d\n",
           k, num_candidates, (*temp_track_num), (*reliable_track_num));
    fflush(stdout);
"@
$p = "d:\DSP\6678\track\track_1\new_reliable.c"
$c = [System.IO.File]::ReadAllText($p)
if ($c.IndexOf($old) -lt 0) { Write-Host "OLD NOT FOUND"; exit 1 }
$c2 = $c.Replace($old, $new)
if ($c2 -eq $c) { Write-Host "NO CHANGE"; exit 2 }
[System.IO.File]::WriteAllText($p, $c2)
Write-Host "OK written. Size =" (Get-Item $p).Length
# quick sanity: 'after_while' line MUST now be outside while body. Run python checker
python -c "
import sys
with open(r'$p','r',encoding='utf-8') as f: c=f.read()
# while-open match close
m = c.find('while(candidates_left')
op = c.find('{', m)
d=1; p=op+1; cl=None
while p<len(c):
    if c[p]=='{': d+=1
    elif c[p]=='}':
        d-=1
        if d==0:
            cl=p; break
    p+=1
print('while open L', c[:op].count(chr(10))+1, '-> matching close L', c[:cl].count(chr(10))+1)
# All 4 diagnostic keys: must be AFTER close (not between break & while-close)
br = c.find('if(!found_any) break;')
print('break line:', c[:br].count(chr(10))+1)
for key in ['after_while','writeback_start','copy cand','wrote_back']:
    kp = c.find(key)
    ln = c[:kp].count(chr(10))+1
    inside = br < kp < cl
    print('  {!r:20s} L{} inside_while(break-close zone)={}'.format(key, ln, inside))
# Brace count
print('braces net:', c.count('{')-c.count('}'))
"
Remove-Item "d:\DSP\6678\track\track_1\_fx21.ps1" -Force
